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
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.vision.PoseValidationResult;
import frc.robot.util.vision.RejectedPose;
import frc.robot.util.vision.VisionPoseValidator;
import java.util.Arrays;

public class VisionSubsystem extends SubsystemBase {

  private final CommandSwerveDrivetrain swerve;
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;
  private final VisionPoseValidator[] validators;

  // Odometry initialization
  private int stablePoseCounter = 5;
  private boolean odometryInitialized = false;

  // Best high-confidence multi-tag pose this cycle — candidate for drift
  // reseeding
  private PoseObservation bestReseedCandidate = null;

  // Round-robin: only process one camera per periodic() to avoid loop overruns
  private int nextCameraIndex = 0;

  // Pre-allocated pose buffer — avoids per-cycle allocation on RoboRIO v1
  private Pose3d[] acceptedPoseBuffer;
  private int acceptedPoseCount = 0;

  public VisionSubsystem(CommandSwerveDrivetrain swerve, VisionConsumer consumer, VisionIO... io) {
    this.swerve = swerve;
    this.consumer = consumer;
    this.io = io;

    inputs = new VisionIOInputs[io.length];
    disconnectedAlerts = new Alert[io.length];
    validators = new VisionPoseValidator[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputs();
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + io[i].getCameraName() + " is disconnected.", AlertType.kWarning);
      validators[i] = new VisionPoseValidator();
    }

    // Pre-allocate: each camera produces at most ~2 poses per cycle
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
    DogLog.time("Perf/Vision");
    acceptedPoseCount = 0;
    bestReseedCandidate = null;

    // Round-robin: process 2 cameras per cycle to reduce loop overhead while
    // maintaining ~25Hz effective update rate per camera (4 cameras at 50Hz).
    int camerasThisCycle = Math.min(2, io.length);
    for (int i = 0; i < camerasThisCycle; i++) {
      int cameraIndex = nextCameraIndex;
      nextCameraIndex = (nextCameraIndex + 1) % io.length;

      io[cameraIndex].updateInputs(inputs[cameraIndex]);
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].isConnected());

      String camKey = "Vision/" + inputs[cameraIndex].getCameraName();
      VisionPoseValidator validator = validators[cameraIndex];
      for (var observation : inputs[cameraIndex].getPoseObservations()) {
        Pose2d visionPose = observation.pose().toPose2d();

        // Pre-check against odometry BEFORE running the validator so that erratic
        // poses never contaminate the validator's inter-frame jump baseline.
        if (odometryInitialized) {
          double discrepancy =
              swerve.getState().Pose.getTranslation().getDistance(visionPose.getTranslation());
          if (discrepancy > MAX_POSE_DISCREPANCY_METERS) {
            continue;
          }
        }

        PoseValidationResult result = validator.validatePose(observation);
        if (result instanceof RejectedPose rejected) {
          if (!odometryInitialized) stablePoseCounter = 5;
          DogLog.log(camKey + "/RejectedPose/Reason", rejected.reason().name());
          DogLog.log(camKey + "/RejectedPose/Details", rejected.details());
          continue;
        }

        // Write into pre-allocated buffer — grow only if needed (rare)
        if (acceptedPoseCount >= acceptedPoseBuffer.length) {
          acceptedPoseBuffer = Arrays.copyOf(acceptedPoseBuffer, acceptedPoseBuffer.length * 2);
        }
        acceptedPoseBuffer[acceptedPoseCount++] = observation.pose();

        // Odometry initialization: require N stable poses before first reset
        if (!odometryInitialized) {
          stablePoseCounter--;
          if (stablePoseCounter <= 0) {
            swerve.resetPose(visionPose);
            markOdometryInitialized();
          }
        }

        consumer.accept(
            visionPose,
            Utils.fpgaToCurrentTime(observation.timestamp()),
            calculateStdDevs(observation, validator));

        // Track best multi-tag pose as a drift-reseed candidate
        if (observation.tagCount() >= POSE_RESEED_MIN_TAG_COUNT) {
          if (bestReseedCandidate == null
              || observation.ambiguity() < bestReseedCandidate.ambiguity()) {
            bestReseedCandidate = observation;
          }
        }
      }
    }

    // Single summary log per cycle — saves ~3-6ms vs per-observation logging
    if (acceptedPoseCount > 0) {
      DogLog.log("Vision/AcceptedPoses", Arrays.copyOf(acceptedPoseBuffer, acceptedPoseCount));
    }
    DogLog.log("Vision/AcceptedPoseCount", acceptedPoseCount);
    DogLog.timeEnd("Perf/Vision");
  }

  private Matrix<N3, N1> calculateStdDevs(PoseObservation obs, VisionPoseValidator validator) {
    double dist = obs.averageTagDistance();
    int tagCount = Math.max(obs.tagCount(), 1);
    // Quadratic distance scaling: trust drops sharply with range; multiple tags
    // reduce uncertainty
    double distanceFactor = (dist * dist) / tagCount;
    double linearStdDev = LINEAR_STDDEV_BASELINE * (1.0 + distanceFactor) * (1.0 + obs.ambiguity());
    // Distrust rotation for single-tag or coplanar multi-tag observations — both
    // have 180° rotational ambiguity (flip-vulnerable). Only true multi-tag
    // observations (tags on different planes) provide a reliable rotational
    // constraint.
    double angularStdDev =
        validator.isEffectivelySingleTag(obs)
            ? Double.MAX_VALUE
            : ANGULAR_STDDEV_BASELINE * (1.0 + distanceFactor);
    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }

  /**
   * If odometry has drifted more than {@code POSE_RESEED_THRESHOLD_METERS} from the best
   * high-confidence vision fix this cycle, hard-resets the pose estimator. Call from {@code
   * Superstructure.periodic()}.
   *
   * @return {@code true} if a reseed was performed
   */
  public boolean tryReseedFromVision(Pose2d currentPose) {
    if (bestReseedCandidate == null) return false;

    Pose2d visionPose = bestReseedCandidate.pose().toPose2d();
    double drift = currentPose.getTranslation().getDistance(visionPose.getTranslation());

    if (drift > POSE_RESEED_THRESHOLD_METERS) {
      swerve.resetPose(visionPose);
      markOdometryInitialized();
      DogLog.log("Vision/PoseReseed/Triggered", true);
      DogLog.log("Vision/PoseReseed/DriftMeters", drift);
      DogLog.log("Vision/PoseReseed/NewPose", visionPose);
      return true;
    }

    return false;
  }

  /**
   * Unconditionally reseeds from the best high-confidence vision fix this cycle. For
   * operator-triggered recovery when odometry has gone badly wrong.
   *
   * @return {@code true} if a qualifying pose was available
   */
  public boolean forceReseedFromVision() {
    if (bestReseedCandidate == null) return false;

    Pose2d visionPose = bestReseedCandidate.pose().toPose2d();
    swerve.resetPose(visionPose);
    markOdometryInitialized();
    DogLog.log("Vision/PoseReseed/ForcedByOperator", true);
    DogLog.log("Vision/PoseReseed/NewPose", visionPose);
    return true;
  }

  private void markOdometryInitialized() {
    odometryInitialized = true;
    stablePoseCounter = 0;
    DogLog.log("Vision/OdometryInitialized", true);
  }

  /**
   * Resets the odometry-initialized flag so that vision will re-seed the pose estimator from
   * scratch on the next accepted observation. Call this at the start of autonomous so vision
   * re-confirms the robot's starting pose from tags rather than carrying over a potentially stale
   * teleop pose.
   */
  public void resetOdometryInitialized() {
    odometryInitialized = false;
    stablePoseCounter = 5;
    DogLog.log("Vision/OdometryInitialized", false);
  }
}
