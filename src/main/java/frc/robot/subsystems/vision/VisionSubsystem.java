// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.VisionConstants.*;

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
import java.util.LinkedList;

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
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  private sealed interface VisionResult permits ValidPose, InvalidPose, NoTargets {}

  private record ValidPose(Pose2d pose, double confidence) implements VisionResult {}

  private record InvalidPose(String reason) implements VisionResult {}

  private record NoTargets() implements VisionResult {}

  @Override
  public void periodic() {
    // Use local variables to reduce field access (optimization)
    final var cameraIOs = io;
    final var cameraInputs = inputs;

    // Initialize logging values
    // LinkedList<Pose3d> allTagPoses = new LinkedList<>();
    // LinkedList<Pose3d> allRobotPoses = new LinkedList<>();
    LinkedList<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    // LinkedList<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // This method will be called once per scheduler run
    for (int cameraIndex = 0; cameraIndex < cameraIOs.length; cameraIndex++) {
      cameraIOs[cameraIndex].updateInputs(cameraInputs[cameraIndex]);

      var result = processCameraData(cameraIndex, cameraInputs[cameraIndex]);

      // Efficient collection operations
      // allTagPoses.addAll(result.tagPoses());
      // allRobotPoses.addAll(result.robotPoses());
      allRobotPosesAccepted.addAll(result.acceptedPoses());
      // allRobotPosesRejected.addAll(result.rejectedPoses());
    }

    // DogLog.log("Vision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
    // DogLog.log("Vision/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
    DogLog.log("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose3d[]::new));
    // DogLog.log("Vision/Summary/RobotPosesRejected",
    // allRobotPosesRejected.toArray(Pose3d[]::new));
  }

  // Record for camera processing results
  private record CameraProcessingResult(
      LinkedList<Pose3d> tagPoses,
      LinkedList<Pose3d> robotPoses,
      LinkedList<Pose3d> acceptedPoses,
      LinkedList<Pose3d> rejectedPoses) {}

  private CameraProcessingResult processCameraData(int cameraIndex, VisionIOInputs inputs) {
    // Update disconnected alert
    disconnectedAlerts[cameraIndex].set(!inputs.isConnected());

    var tagPoses = new LinkedList<Pose3d>();
    var robotPoses = new LinkedList<Pose3d>();
    var acceptedPoses = new LinkedList<Pose3d>();
    var rejectedPoses = new LinkedList<Pose3d>();

    // Add tag poses
    for (int tagId : inputs.getTagIds()) {
      APTAG_FIELD_LAYOUT.getTagPose(tagId).ifPresent(tagPoses::add);
    }

    // Process pose observations with sealed classes
    for (var observation : inputs.getPoseObservations()) {
      robotPoses.add(observation.pose());

      // Use instanceof with pattern matching
      var validationResult = poseValidator.validatePose(observation);
      String cameraIndexString = "Vision/Camera" + cameraIndex;

      if (validationResult instanceof AcceptedPose accepted) {
        handleAcceptedPose(accepted, cameraIndexString, cameraIndex, acceptedPoses);
      } else if (validationResult instanceof RejectedPose rejected) {
        handleRejectedPose(rejected, cameraIndexString, rejectedPoses);
      }
    }

    return new CameraProcessingResult(tagPoses, robotPoses, acceptedPoses, rejectedPoses);
  }

  private void handleAcceptedPose(
      AcceptedPose accepted,
      String cameraIndexString,
      int cameraIndex,
      LinkedList<Pose3d> acceptedPoses) {
    acceptedPoses.add(accepted.poseObservation().pose());

    // Check if odometry is initialized
    if (!odometryInitialized) {
      stablePoseCounter--;
      if (stablePoseCounter <= 0) {
        swerve.resetPose(accepted.poseObservation().pose().toPose2d());
        odometryInitialized = true;
        DogLog.log("Vision/OdometryInitialized", true);
      }
    }

    // Add to pose estimator
    var stdDevs = calculateStandardDeviations(accepted);
    consumer.accept(
        accepted.poseObservation().pose().toPose2d(),
        Utils.fpgaToCurrentTime(accepted.poseObservation().timestamp()),
        stdDevs);
  }

  private void handleRejectedPose(
      RejectedPose rejected, String cameraIndexString, LinkedList<Pose3d> rejectedPoses) {
    // If odometry not initialized, reset stable pose counter
    if (!odometryInitialized) {
      stablePoseCounter = 5;
    }

    rejectedPoses.add(rejected.poseObservation().pose());
  }

  private Matrix<N3, N1> calculateStandardDeviations(AcceptedPose accepted) {
    // Your existing standard deviation calculation logic
    double stdDevFactor =
        (1 + accepted.poseObservation().averageTagDistance())
            * (1 + accepted.poseObservation().ambiguity())
            / Math.sqrt(Math.max(accepted.poseObservation().tagCount(), 1));
    double linearStdDev = LINEAR_STDDEV_BASELINE * stdDevFactor;
    double angularStdDev = ANGULAR_STDDEV_BASELINE * stdDevFactor;

    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }
}
