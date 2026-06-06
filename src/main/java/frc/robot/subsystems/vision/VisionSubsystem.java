// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.util.constants.VisionConstants.*;

import com.ctre.phoenix6.Utils;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.vision.PoseValidationResult;
import frc.robot.util.vision.RejectedPose;
import frc.robot.util.vision.VisionPoseValidator;
import java.util.Arrays;
import java.util.Optional;

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

  // Pre-allocated pose buffer
  private Pose3d[] acceptedPoseBuffer;
  private int acceptedPoseCount = 0;

  // Per-camera accepted pose logging buffers.
  private Pose3d[][] acceptedPoseByCameraBuffer;
  private int[] acceptedPoseByCameraCount;

  // Most recent accepted pose per camera for cross-camera consistency checks.
  private Pose2d[] lastAcceptedPoseByCamera;
  private double[] lastAcceptedTimestampByCamera;

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

      if (io[i] instanceof VisionIOPhotonVision photonVisionIo) {
        photonVisionIo.setHeadingProvider(new DrivetrainHeadingProvider());
      }
    }

    // Pre-allocate: each camera produces at most ~2 poses per cycle
    acceptedPoseBuffer = new Pose3d[io.length * 2];
    acceptedPoseByCameraBuffer = new Pose3d[io.length][];
    acceptedPoseByCameraCount = new int[io.length];
    lastAcceptedPoseByCamera = new Pose2d[io.length];
    lastAcceptedTimestampByCamera = new double[io.length];
    for (int i = 0; i < io.length; i++) {
      acceptedPoseByCameraBuffer[i] = new Pose3d[2];
      lastAcceptedTimestampByCamera[i] = -1.0;
    }
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
    Arrays.fill(acceptedPoseByCameraCount, 0);
    bestReseedCandidate = null;

    // Process all cameras every cycle — RoboRIO v2 (866 MHz, 512 MB) has
    // sufficient headroom. Each camera costs ~0.5ms (PnP runs on coprocessor).
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
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

        if (!passesCrossCameraConsistency(cameraIndex, visionPose, observation.timestamp())) {
          if (!odometryInitialized) stablePoseCounter = 5;
          continue;
        }

        // Write per-camera accepted poses for observability.
        if (acceptedPoseByCameraCount[cameraIndex]
            >= acceptedPoseByCameraBuffer[cameraIndex].length) {
          acceptedPoseByCameraBuffer[cameraIndex] =
              Arrays.copyOf(
                  acceptedPoseByCameraBuffer[cameraIndex],
                  acceptedPoseByCameraBuffer[cameraIndex].length * 2);
        }
        acceptedPoseByCameraBuffer[cameraIndex][acceptedPoseByCameraCount[cameraIndex]++] =
            observation.pose();

        if (!passesCrossCameraConsistency(cameraIndex, visionPose, observation.timestamp())) {
          if (!odometryInitialized) stablePoseCounter = 5;
          continue;
        }

        if (!passesCoplanarHistoricalYawConsistency(
            cameraIndex, visionPose, validator, observation)) {
          if (!odometryInitialized) stablePoseCounter = 5;
          continue;
        }

        if (!passesCoplanarYawConsistency(cameraIndex, visionPose, validator, observation)) {
          if (!odometryInitialized) stablePoseCounter = 5;
          continue;
        }

        // Write per-camera accepted poses for observability.
        if (acceptedPoseByCameraCount[cameraIndex]
            >= acceptedPoseByCameraBuffer[cameraIndex].length) {
          acceptedPoseByCameraBuffer[cameraIndex] =
              Arrays.copyOf(
                  acceptedPoseByCameraBuffer[cameraIndex],
                  acceptedPoseByCameraBuffer[cameraIndex].length * 2);
        }
        acceptedPoseByCameraBuffer[cameraIndex][acceptedPoseByCameraCount[cameraIndex]++] =
            observation.pose();

        // Write into pre-allocated buffer — grow only if needed (rare)
        if (acceptedPoseCount >= acceptedPoseBuffer.length) {
          acceptedPoseBuffer = Arrays.copyOf(acceptedPoseBuffer, acceptedPoseBuffer.length * 2);
        }
        acceptedPoseBuffer[acceptedPoseCount++] = observation.pose();

        lastAcceptedPoseByCamera[cameraIndex] = visionPose;
        lastAcceptedTimestampByCamera[cameraIndex] = observation.timestamp();

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
            calculateStdDevs(observation, validator, cameraIndex));

        // Track best multi-tag pose as a drift-reseed candidate
        if (observation.tagCount() >= POSE_RESEED_MIN_TAG_COUNT) {
          if (bestReseedCandidate == null
              || observation.ambiguity() < bestReseedCandidate.ambiguity()) {
            bestReseedCandidate = observation;
          }
        }
      }
    }

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      String camKey = "Vision/" + inputs[cameraIndex].getCameraName();
      DogLog.log(camKey + "/AcceptedPoseCount", acceptedPoseByCameraCount[cameraIndex]);
      if (acceptedPoseByCameraCount[cameraIndex] > 0) {
        DogLog.log(
            camKey + "/AcceptedPoses",
            Arrays.copyOf(
                acceptedPoseByCameraBuffer[cameraIndex], acceptedPoseByCameraCount[cameraIndex]));
      } else {
        DogLog.log(camKey + "/AcceptedPoses", new Pose3d[0]);
      }
    }

    // Single summary log per cycle.
    DogLog.log("Vision/AcceptedPoseCount", acceptedPoseCount);
    if (acceptedPoseCount > 0) {
      DogLog.log("Vision/AcceptedPoses", Arrays.copyOf(acceptedPoseBuffer, acceptedPoseCount));
    } else {
      DogLog.log("Vision/AcceptedPoses", new Pose3d[0]);
    }
    DogLog.timeEnd("Perf/Vision");
  }

  private boolean passesCrossCameraConsistency(
      int cameraIndex, Pose2d candidatePose, double candidateTimestamp) {
    String camKey = "Vision/" + inputs[cameraIndex].getCameraName();
    for (int otherIndex = 0; otherIndex < io.length; otherIndex++) {
      if (otherIndex == cameraIndex) continue;
      if (lastAcceptedPoseByCamera[otherIndex] == null) continue;

      double age = Math.abs(candidateTimestamp - lastAcceptedTimestampByCamera[otherIndex]);
      if (age > CROSS_CAMERA_MAX_AGE_SECONDS) continue;

      double discrepancy =
          candidatePose
              .getTranslation()
              .getDistance(lastAcceptedPoseByCamera[otherIndex].getTranslation());
      if (discrepancy > MAX_CROSS_CAMERA_DISCREPANCY_METERS) {
        DogLog.log(camKey + "/RejectedPose/Reason", "CROSS_CAMERA_DISCREPANCY");
        DogLog.log(
            camKey + "/RejectedPose/Details",
            "Against "
                + inputs[otherIndex].getCameraName()
                + ", discrepancy: "
                + discrepancy
                + ", age: "
                + age);
        return false;
      }
    }
    return true;
  }

  private boolean passesCoplanarYawConsistency(
      int cameraIndex,
      Pose2d candidatePose,
      VisionPoseValidator validator,
      PoseObservation observation) {
    if (!validator.isEffectivelySingleTag(observation)) {
      return true;
    }

    if (swerve.samplePoseAt(observation.timestamp()).isPresent()) {
      return true;
    }

    String camKey = "Vision/" + inputs[cameraIndex].getCameraName();
    double yawErrorRad =
        Math.abs(
            MathUtil.angleModulus(
                candidatePose.getRotation().getRadians()
                    - swerve.getState().Pose.getRotation().getRadians()));
    double maxYawErrorRad = Math.toRadians(COPLANAR_MAX_YAW_DISCREPANCY_DEG);

    if (yawErrorRad > maxYawErrorRad) {
      DogLog.log(camKey + "/RejectedPose/Reason", "COPLANAR_YAW_DISCREPANCY");
      DogLog.log(
          camKey + "/RejectedPose/Details",
          "yaw error deg: "
              + Math.toDegrees(yawErrorRad)
              + " > max: "
              + COPLANAR_MAX_YAW_DISCREPANCY_DEG);
      return false;
    }

    return true;
  }

  private boolean passesCoplanarHistoricalYawConsistency(
      int cameraIndex,
      Pose2d candidatePose,
      VisionPoseValidator validator,
      PoseObservation observation) {
    if (!validator.isEffectivelySingleTag(observation)) {
      return true;
    }

    Optional<Pose2d> historicalPoseOpt = swerve.samplePoseAt(observation.timestamp());
    if (historicalPoseOpt.isEmpty()) {
      return true;
    }

    String camKey = "Vision/" + inputs[cameraIndex].getCameraName();
    Pose2d historicalPose = historicalPoseOpt.get();

    double yawErrorRad =
        Math.abs(
            MathUtil.angleModulus(
                candidatePose.getRotation().getRadians()
                    - historicalPose.getRotation().getRadians()));
    double maxYawErrorRad = Math.toRadians(COPLANAR_HISTORICAL_MAX_YAW_DISCREPANCY_DEG);

    if (yawErrorRad > maxYawErrorRad) {
      DogLog.log(camKey + "/RejectedPose/Reason", "COPLANAR_HISTORICAL_YAW_DISCREPANCY");
      DogLog.log(
          camKey + "/RejectedPose/Details",
          "historical yaw error deg: "
              + Math.toDegrees(yawErrorRad)
              + " > max: "
              + COPLANAR_HISTORICAL_MAX_YAW_DISCREPANCY_DEG);
      return false;
    }

    return true;
  }

  private class DrivetrainHeadingProvider implements VisionIOPhotonVision.VisionHeadingProvider {
    @Override
    public Optional<Rotation2d> getHeadingAtTimestamp(double fpgaTimestampSeconds) {
      return swerve.samplePoseAt(fpgaTimestampSeconds).map(Pose2d::getRotation);
    }

    @Override
    public Optional<Pose3d> getSeedPoseAtTimestamp(double fpgaTimestampSeconds) {
      return swerve.samplePoseAt(fpgaTimestampSeconds).map(Pose3d::new);
    }

    @Override
    public double getAngularRateRadPerSec() {
      return swerve.getState().Speeds.omegaRadiansPerSecond;
    }
  }

  private Matrix<N3, N1> calculateStdDevs(
      PoseObservation obs, VisionPoseValidator validator, int cameraIndex) {
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

    int factorIndex = Math.min(cameraIndex, CAMERA_STDDEV_FACTORS.length - 1);
    double cameraFactor = CAMERA_STDDEV_FACTORS[factorIndex];
    linearStdDev *= cameraFactor;
    if (angularStdDev != Double.MAX_VALUE) {
      angularStdDev *= cameraFactor;
    }

    if (obs.type() == PoseObservationType.MEGATAG_2) {
      linearStdDev *= LINEAR_STDDEV_MEGATAG2_FACTOR;
      if (angularStdDev != Double.MAX_VALUE) {
        angularStdDev *= ANGULAR_STDDEV_MEGATAG2_ANGLE_FACTOR;
      }
    }

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
