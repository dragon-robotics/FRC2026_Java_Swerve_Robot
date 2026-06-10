// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT;
import static frc.robot.util.constants.VisionConstants.AIM_LINEAR_STDDEV_MULTIPLIER;
import static frc.robot.util.constants.VisionConstants.APPLY_COPLANAR_PENALTY;
import static frc.robot.util.constants.VisionConstants.CAMERA_STDDEV_FACTORS;
import static frc.robot.util.constants.VisionConstants.COPLANAR_ANGLE_THRESHOLD_DEG;
import static frc.robot.util.constants.VisionConstants.HEADING_STDDEV_IGNORE;
import static frc.robot.util.constants.VisionConstants.LINEAR_STDDEV_BASELINE;
import static frc.robot.util.constants.VisionConstants.MAX_AMBIGUITY;
import static frc.robot.util.constants.VisionConstants.MAX_AVG_TAG_DISTANCE_METERS;
import static frc.robot.util.constants.VisionConstants.MAX_POSE_DELTA_METERS;
import static frc.robot.util.constants.VisionConstants.MAX_Z_ERROR;
import static frc.robot.util.constants.VisionConstants.SINGLE_TAG_LINEAR_STDDEV_MULTIPLIER;

import com.ctre.phoenix6.Utils;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * Lean AprilTag pose-estimation subsystem.
 *
 * <p>Design (see {@code docs/superpowers/specs/2026-06-08-vision-rewrite-design.md}):
 *
 * <ul>
 *   <li>Vision NEVER hard-resets the drivetrain pose during normal operation. It only feeds
 *       weighted measurements through {@link VisionConsumer}; the pose estimator blends them.
 *   <li>Vision heading is ignored (huge angular std-dev); the gyro is authoritative.
 *   <li>Translation trust scales with distance and tightens while aiming.
 *   <li>Simple, readable rejection: tag count, Z, field bounds, single-tag ambiguity, max distance.
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {

  /** Snapshots older than this are treated as stale by the dashboard accessor. */
  private static final double SNAPSHOT_MAX_AGE_SECONDS = 0.5;

  private final CommandSwerveDrivetrain swerve;
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;

  /** True while the robot is actively aiming/aligning to score — tightens translation trust. */
  private boolean aiming = false;

  /** Most recent accepted observation across all cameras (for the dashboard overlay). */
  private Pose2d lastAcceptedPose = null;

  private int[] lastAcceptedTagIDs = new int[0];
  private double lastAcceptedTimestamp = -1.0;

  public VisionSubsystem(CommandSwerveDrivetrain swerve, VisionConsumer consumer, VisionIO... io) {
    this.swerve = swerve;
    this.consumer = consumer;
    this.io = io;

    inputs = new VisionIOInputs[io.length];
    disconnectedAlerts = new Alert[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputs();
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + io[i].getCameraName() + " is disconnected.", AlertType.kWarning);

      if (io[i] instanceof VisionIOPhotonVision photonVisionIo) {
        photonVisionIo.setHeadingProvider(new DrivetrainHeadingProvider());
      }
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /** Immutable snapshot of the latest accepted observation, consumed by the dashboard overlay. */
  public static record AcceptedObservationSnapshot(Pose2d pose, int[] tagIDs, double timestamp) {}

  /** Sets whether the robot is actively aiming/aligning (tightens vision translation trust). */
  public void setAiming(boolean aiming) {
    this.aiming = aiming;
  }

  @Override
  public void periodic() {
    DogLog.time("Perf/Vision");

    List<Pose3d> acceptedPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      io[cameraIndex].updateInputs(inputs[cameraIndex]);
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].isConnected());

      String camKey = "Vision/" + inputs[cameraIndex].getCameraName();

      PoseObservation[] observations = inputs[cameraIndex].getPoseObservations();

      // Raw pre-filter observation logging so real match logs can be replayed through
      // the
      // filter offline (see VisionFilterStabilityTest /
      // scripts/analyze_vision_stability.py).
      logRawObservations(camKey, observations);

      for (PoseObservation observation : observations) {
        Optional<String> rejection = rejectionReason(observation);
        if (rejection.isPresent()) {
          rejectedPoses.add(observation.pose());
          DogLog.log(camKey + "/RejectedReason", rejection.get());
          continue;
        }

        Pose2d pose2d = observation.pose().toPose2d();
        Pose2d referencePose =
            swerve.samplePoseAt(observation.timestamp()).orElse(swerve.getState().Pose);
        double innovationMeters =
            pose2d.getTranslation().getDistance(referencePose.getTranslation());
        if (innovationMeters > MAX_POSE_DELTA_METERS) {
          rejectedPoses.add(observation.pose());
          DogLog.log(camKey + "/RejectedReason", "POSE_DELTA=" + innovationMeters);
          continue;
        }

        acceptedPoses.add(observation.pose());

        consumer.accept(
            pose2d,
            Utils.fpgaToCurrentTime(observation.timestamp()),
            standardDeviations(observation, cameraIndex));

        if (observation.timestamp() > lastAcceptedTimestamp) {
          lastAcceptedPose = pose2d;
          lastAcceptedTagIDs = Arrays.copyOf(observation.tagIDs(), observation.tagIDs().length);
          lastAcceptedTimestamp = observation.timestamp();
        }
      }
    }

    logPoseArray("Vision/AcceptedPoses", acceptedPoses);
    logPoseArray("Vision/RejectedPoses", rejectedPoses);
    DogLog.log("Vision/Aiming", aiming);
    DogLog.timeEnd("Perf/Vision");
  }

  /**
   * Returns a human-readable rejection reason, or empty if the observation should be accepted.
   *
   * <p>Reject when: no tags, unrealistic Z, outside the field, a single tag with high ambiguity, or
   * the average tag distance exceeds {@link
   * frc.robot.util.constants.VisionConstants#MAX_AVG_TAG_DISTANCE_METERS}.
   *
   * <p>Static and package-private so tests can exercise the real gate logic without a HAL/sim
   * drivetrain.
   */
  static Optional<String> rejectionReason(PoseObservation observation) {
    if (observation.tagCount() == 0) {
      return Optional.of("NO_TAGS");
    }

    Pose3d pose = observation.pose();
    if (Math.abs(pose.getZ()) > MAX_Z_ERROR) {
      return Optional.of("Z=" + pose.getZ());
    }

    Pose2d pose2d = pose.toPose2d();
    if (pose2d.getX() < 0.0
        || pose2d.getX() > APTAG_FIELD_LAYOUT.getFieldLength()
        || pose2d.getY() < 0.0
        || pose2d.getY() > APTAG_FIELD_LAYOUT.getFieldWidth()) {
      return Optional.of("OUT_OF_BOUNDS");
    }

    if (observation.tagCount() == 1 && observation.ambiguity() > MAX_AMBIGUITY) {
      return Optional.of("AMBIGUITY=" + observation.ambiguity());
    }

    if (observation.averageTagDistance() > MAX_AVG_TAG_DISTANCE_METERS) {
      return Optional.of("DISTANCE=" + observation.averageTagDistance());
    }

    return Optional.empty();
  }

  /**
   * Distance-scaled translation std-dev with heading ignored. Translation trust tightens while
   * aiming. Mirrors the AdvantageKit model with 1678's heading-ignore strategy.
   */
  private Matrix<N3, N1> standardDeviations(PoseObservation observation, int cameraIndex) {
    return standardDeviations(observation, cameraIndex, aiming);
  }

  /**
   * Pure standard-deviation model with an explicit aiming flag. Package-private so tests can
   * exercise the exact production std-dev math without depending on subsystem state.
   */
  static Matrix<N3, N1> standardDeviations(
      PoseObservation observation, int cameraIndex, boolean aiming) {
    double tagCount = Math.max(observation.tagCount(), 1);
    double distance = observation.averageTagDistance();
    double factor = (distance * distance) / tagCount;

    double cameraFactor =
        CAMERA_STDDEV_FACTORS[Math.min(cameraIndex, CAMERA_STDDEV_FACTORS.length - 1)];
    double aimFactor = aiming ? AIM_LINEAR_STDDEV_MULTIPLIER : 1.0;
    // Coplanar multi-tag observations have the same 180° flip ambiguity as
    // single-tag PnP.
    // Apply the single-tag std-dev penalty to prevent a wrong mirror solution from
    // being
    // trusted with full multi-tag confidence.
    boolean coplanarPenaltyApplies =
        APPLY_COPLANAR_PENALTY && areTagsCoplanar(observation.tagIDs());
    double singleTagFactor =
        (observation.tagCount() == 1 || coplanarPenaltyApplies)
            ? SINGLE_TAG_LINEAR_STDDEV_MULTIPLIER
            : 1.0;

    double linearStdDev =
        LINEAR_STDDEV_BASELINE * factor * cameraFactor * aimFactor * singleTagFactor;

    return VecBuilder.fill(linearStdDev, linearStdDev, HEADING_STDDEV_IGNORE);
  }

  // ────────────────────────────────────────────────────────────────────────────
  // Coplanar detection
  // ────────────────────────────────────────────────────────────────────────────

  /**
   * Returns {@code true} when all tags in {@code tagIDs} lie on the same flat surface (same Hub
   * face). Coplanar multi-tag PnP has the same 180° rotational ambiguity as single-tag PnP — the
   * planar geometry admits two mirror solutions. These observations must receive the single-tag
   * std-dev penalty even though {@code tagCount ≥ 2}.
   *
   * <p>Detection: compare the outward Z-axis (normal) of each tag's field pose. Tags are coplanar
   * when all normals are within {@link
   * frc.robot.util.constants.VisionConstants#COPLANAR_ANGLE_THRESHOLD_DEG} of the first tag's
   * normal.
   *
   * <p>Package-private so tests can call it directly.
   */
  static boolean areTagsCoplanar(int[] tagIDs) {
    if (tagIDs == null || tagIDs.length <= 1) {
      return true; // single tag is trivially "coplanar"
    }
    var firstOpt = APTAG_FIELD_LAYOUT.getTagPose(tagIDs[0]);
    if (firstOpt.isEmpty()) {
      return true; // unknown tag — treat as vulnerable
    }
    Rotation3d referenceNormal = firstOpt.get().getRotation();
    double thresholdRad = Math.toRadians(COPLANAR_ANGLE_THRESHOLD_DEG);
    for (int i = 1; i < tagIDs.length; i++) {
      var tagOpt = APTAG_FIELD_LAYOUT.getTagPose(tagIDs[i]);
      if (tagOpt.isEmpty()) {
        continue; // unknown tag — skip
      }
      Rotation3d diff = tagOpt.get().getRotation().minus(referenceNormal);
      if (Math.abs(diff.getAngle()) > thresholdRad) {
        return false; // tags face different directions → not coplanar
      }
    }
    return true;
  }

  /** Logs a pose list as a struct array for AdvantageScope. */
  @SuppressWarnings("null") // DogLog null-annotation interop on Pose3d[] is benign.
  private static void logPoseArray(String key, List<Pose3d> poses) {
    DogLog.log(key, poses.toArray(new Pose3d[0]));
  }

  /**
   * Logs the raw, pre-filter observations for a camera as index-aligned scalar arrays. This
   * captures everything the rejection and std-dev logic consume, so a real match log can be
   * replayed through the filter offline to diagnose acceptance and pose-jump behavior.
   */
  @SuppressWarnings("null") // DogLog null-annotation interop on Pose3d[] is benign.
  private static void logRawObservations(String camKey, PoseObservation[] observations) {
    Pose3d[] poses = new Pose3d[observations.length];
    double[] timestamps = new double[observations.length];
    double[] ambiguities = new double[observations.length];
    double[] tagCounts = new double[observations.length];
    double[] avgDistances = new double[observations.length];
    for (int i = 0; i < observations.length; i++) {
      poses[i] = observations[i].pose();
      timestamps[i] = observations[i].timestamp();
      ambiguities[i] = observations[i].ambiguity();
      tagCounts[i] = observations[i].tagCount();
      avgDistances[i] = observations[i].averageTagDistance();
    }
    DogLog.log(camKey + "/RawObs/Poses", poses);
    DogLog.log(camKey + "/RawObs/Timestamp", timestamps);
    DogLog.log(camKey + "/RawObs/Ambiguity", ambiguities);
    DogLog.log(camKey + "/RawObs/TagCount", tagCounts);
    DogLog.log(camKey + "/RawObs/AvgDistance", avgDistances);
  }

  /** Returns a consistent snapshot of the latest accepted observation, if recent. */
  public Optional<AcceptedObservationSnapshot> getLatestAcceptedObservationSnapshot() {
    if (lastAcceptedPose == null
        || (Timer.getFPGATimestamp() - lastAcceptedTimestamp) > SNAPSHOT_MAX_AGE_SECONDS) {
      return Optional.empty();
    }
    return Optional.of(
        new AcceptedObservationSnapshot(
            lastAcceptedPose,
            Arrays.copyOf(lastAcceptedTagIDs, lastAcceptedTagIDs.length),
            lastAcceptedTimestamp));
  }

  /**
   * Operator-triggered recovery: snaps the drivetrain pose to the most recent accepted vision pose.
   * This is the ONLY path that resets the pose, and it is never automatic.
   *
   * @return true if a recent accepted pose was available
   */
  public boolean forceReseedFromVision() {
    Optional<AcceptedObservationSnapshot> snapshot = getLatestAcceptedObservationSnapshot();
    if (snapshot.isEmpty()) {
      return false;
    }
    swerve.resetPose(snapshot.get().pose());
    DogLog.log("Vision/ForceReseed", snapshot.get().pose());
    return true;
  }

  /** Feeds the drivetrain heading to PhotonVision for single-tag constrained solving. */
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

    @Override
    public double getLinearSpeedMetersPerSecond() {
      double vx = swerve.getState().Speeds.vxMetersPerSecond;
      double vy = swerve.getState().Speeds.vyMetersPerSecond;
      return Math.hypot(vx, vy);
    }
  }
}
