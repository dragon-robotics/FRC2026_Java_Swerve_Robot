// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.util.constants.VisionConstants.*;

import com.ctre.phoenix6.Utils;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.vision.AcceptedPose;
import frc.robot.util.vision.RejectedPose;
import frc.robot.util.vision.VisionPoseValidator;
import java.util.Arrays;

public class VisionSubsystem extends SubsystemBase {

  private final CommandSwerveDrivetrain swerve;
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;

  // Check for odometry initialization and use about //
  private int stablePoseCounter = 5;
  private boolean odometryInitialized = false;

  // Import VisionPoseValidator to validate our vision observations //
  private final VisionPoseValidator poseValidator = new VisionPoseValidator();

  // Pre-allocated pose buffer — avoids ArrayList and toArray() allocation each cycle.
  // Max capacity = io.length cameras * ~2 poses each (generous upper bound).
  // acceptedPoseCount tracks how many slots are filled this cycle.
  private Pose3d[] acceptedPoseBuffer;
  private int acceptedPoseCount = 0;

  // Pre-computed log key strings — avoid string concatenation each cycle
  private final String[] cameraPerfKeys;

  // Static empty Pose3d array (kept for future use)
  // private static final Pose3d[] EMPTY_POSE3D_ARRAY = new Pose3d[0];

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(CommandSwerveDrivetrain swerve, VisionConsumer consumer, VisionIO... io) {
    this.swerve = swerve;
    this.consumer = consumer;
    this.io = io;

    // Initialize the the inputs
    inputs = new VisionIOInputs[io.length];
    disconnectedAlerts = new Alert[io.length];

    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputs();
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + io[i].getCameraName() + " is disconnected.", AlertType.kWarning);
    }

    // Pre-compute camera log key strings (avoids string concat each cycle)
    cameraPerfKeys = new String[io.length];
    for (int i = 0; i < io.length; i++) {
      cameraPerfKeys[i] = "Vision/Perf/Camera" + i;
    }

    // Pre-allocate pose buffer: each camera can produce ~2 poses max per cycle
    acceptedPoseBuffer = new Pose3d[io.length * 2];
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    DogLog.time("Vision/Perf/TotalPeriodic");

    // Use local variables to reduce field access (optimization)
    final var cameraIOs = io;
    final var cameraInputs = inputs;

    // Reset pose count — no clearing needed, just overwrite slots
    acceptedPoseCount = 0;

    // This method will be called once per scheduler run
    for (int cameraIndex = 0; cameraIndex < cameraIOs.length; cameraIndex++) {
      DogLog.time(cameraPerfKeys[cameraIndex]);

      cameraIOs[cameraIndex].updateInputs(cameraInputs[cameraIndex]);

      processCameraData(cameraIndex, cameraInputs[cameraIndex]);

      DogLog.timeEnd(cameraPerfKeys[cameraIndex]);
    }

    // Log every cycle — use Arrays.copyOf only when we have poses (avoids full-buffer copy)
    // DogLog.log(
    //     "Vision/Summary/RobotPosesAccepted",
    //     acceptedPoseCount == 0
    //         ? EMPTY_POSE3D_ARRAY
    //         : Arrays.copyOf(acceptedPoseBuffer, acceptedPoseCount));

    DogLog.timeEnd("Vision/Perf/TotalPeriodic");
  }

  private void processCameraData(int cameraIndex, VisionIOInputs inputs) {
    // Update disconnected alert
    disconnectedAlerts[cameraIndex].set(!inputs.isConnected());

    // Process pose observations — no intermediate collections needed
    for (var observation : inputs.getPoseObservations()) {

      // Use instanceof with pattern matching
      var validationResult = poseValidator.validatePose(observation);

      if (validationResult instanceof AcceptedPose accepted) {
        handleAcceptedPose(accepted);
      } else if (validationResult instanceof RejectedPose rejected) {
        handleRejectedPose(rejected);
      }
    }
  }

  private void handleAcceptedPose(AcceptedPose accepted) {
    // Cache the poseObservation record accessor to avoid repeated calls
    var poseObs = accepted.poseObservation();

    // Write directly into pre-allocated buffer — grow if needed (rare)
    if (acceptedPoseCount >= acceptedPoseBuffer.length) {
      acceptedPoseBuffer = Arrays.copyOf(acceptedPoseBuffer, acceptedPoseBuffer.length * 2);
    }
    acceptedPoseBuffer[acceptedPoseCount++] = poseObs.pose();

    // Check if odometry is initialized
    if (!odometryInitialized) {
      stablePoseCounter--;
      if (stablePoseCounter <= 0) {
        swerve.resetPose(poseObs.pose().toPose2d());
        odometryInitialized = true;
        DogLog.log("Vision/OdometryInitialized", true);
      }
    }

    // Add to pose estimator
    var stdDevs = calculateStandardDeviations(accepted);
    consumer.accept(
        poseObs.pose().toPose2d(), Utils.fpgaToCurrentTime(poseObs.timestamp()), stdDevs);
  }

  private void handleRejectedPose(RejectedPose rejected) {
    // If odometry not initialized, reset stable pose counter
    if (!odometryInitialized) {
      stablePoseCounter = 5;
    }
  }

  private Matrix<N3, N1> calculateStandardDeviations(AcceptedPose accepted) {
    var poseObs = accepted.poseObservation();
    double stdDevFactor =
        (1 + poseObs.averageTagDistance())
            * (1 + poseObs.ambiguity())
            / Math.sqrt(Math.max(poseObs.tagCount(), 1));
    double linearStdDev = LINEAR_STDDEV_BASELINE * stdDevFactor;
    double angularStdDev = ANGULAR_STDDEV_BASELINE * stdDevFactor;

    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }
}
