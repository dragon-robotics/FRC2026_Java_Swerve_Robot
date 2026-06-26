package frc.robot.util.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * Vision camera transforms, AprilTag acceptance gates, and pose-estimator
 * uncertainty settings.
 *
 * <p>
 * Camera transforms are robot-relative. Translation values are meters in the
 * WPILib robot frame
 * (+X forward, +Y left, +Z up). Rotation values are radians.
 */
public final class VisionConstants {

  /* AprilTag pose-estimation cameras */

  public static final String[] APTAG_CAMERA_NAMES = {
      "AprilTagPoseEstCameraF",
      "AprilTagPoseEstCameraR",
      "AprilTagPoseEstCameraB",
      "AprilTagPoseEstCameraL"
  };

  public static final Transform3d APTAG_POSE_EST_CAM_F_POS = new Transform3d(
      new Translation3d(
          Units.inchesToMeters(-11.152),
          Units.inchesToMeters(-7.579),
          Units.inchesToMeters(20.930)),
      new Rotation3d(0, Units.degreesToRadians(-15), 0));

  public static final Transform3d APTAG_POSE_EST_CAM_R_POS = new Transform3d(
      new Translation3d(
          Units.inchesToMeters(-8.387),
          Units.inchesToMeters(-13.355),
          Units.inchesToMeters(15.931)),
      new Rotation3d(0, Units.degreesToRadians(-12), Units.degreesToRadians(-90)));

  public static final Transform3d APTAG_POSE_EST_CAM_B_POS = new Transform3d(
      new Translation3d(
          Units.inchesToMeters(-9.164),
          Units.inchesToMeters(12.5),
          Units.inchesToMeters(20.839)),
      new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(180)));

  public static final Transform3d APTAG_POSE_EST_CAM_L_POS = new Transform3d(
      new Translation3d(
          Units.inchesToMeters(-8.387),
          Units.inchesToMeters(13.355),
          Units.inchesToMeters(15.931)),
      new Rotation3d(0, Units.degreesToRadians(-12), Units.degreesToRadians(90)));

  public static final Transform3d[] APTAG_POSE_EST_CAM_POSITIONS = {
      APTAG_POSE_EST_CAM_F_POS,
      APTAG_POSE_EST_CAM_R_POS,
      APTAG_POSE_EST_CAM_B_POS,
      APTAG_POSE_EST_CAM_L_POS
  };

  /**
   * Default x, y, and heading standard deviations used when no richer model is
   * available.
   */
  public static final Matrix<N3, N1> DEFAULT_TAG_STDDEV = VecBuilder.fill(0.9, 0.9, 0.9);

  /* Acceptance gates */

  /** Reject single-tag observations whose ambiguity exceeds this. */
  public static final double MAX_AMBIGUITY = 0.2;

  /** Reject poses whose |Z| exceeds this (meters). */
  public static final double MAX_Z_ERROR = 0.5;

  /** Coarse IO per-target prefilter: ignore tags farther than this (meters). */
  public static final double MAX_TAG_DISTANCE = 8.0;

  /**
   * Reject observations whose average tag distance exceeds this (meters). 1678
   * strategy.
   */
  public static final double MAX_AVG_TAG_DISTANCE_METERS = 5.5;

  /**
   * Reject vision updates while chassis pitch/roll exceeds this absolute tilt
   * (degrees).
   */
  public static final double MAX_ABS_TILT_DEGREES_FOR_VISION = Double
      .parseDouble(System.getProperty("vision.maxAbsTiltDeg", "8.0"));

  /**
   * Innovation gate on translation: reject vision poses farther than this from
   * the predicted
   * drivetrain pose at the same timestamp.
   */
  public static final double MAX_POSE_DELTA_METERS = 2.5;

  /* VisionSubsystem state/reseed behavior */

  /** Snapshots older than this are treated as stale by the dashboard accessor. */
  public static final double SNAPSHOT_MAX_AGE_SECONDS = 0.5;

  /** Limit how often disabled auto-reseed can reset pose. */
  public static final double DISABLED_AUTO_RESEED_MIN_INTERVAL_SECONDS = 0.5;

  /** Re-seed again in disabled if odometry drifts too far from latest accepted vision. */
  public static final double DISABLED_AUTO_RESEED_DELTA_METERS = 0.25;

  /** Disabled auto-reseed only trusts multi-tag solutions to avoid gyro-seeded single-tag bias. */
  public static final int DISABLED_AUTO_RESEED_MIN_TAG_COUNT = 2;

  /** Stable MultiTagPnP observations required before vision init is considered complete. */
  public static final int MULTITAG_INIT_STABLE_POSES_REQUIRED = 5;

  /** Maximum translation delta between consecutive MultiTagPnP observations to stay stable. */
  public static final double MULTITAG_INIT_MAX_TRANSLATION_DELTA_METERS = 0.20;

  /** Maximum heading delta between consecutive MultiTagPnP observations to stay stable. */
  public static final double MULTITAG_INIT_MAX_HEADING_DELTA_DEGREES = 10.0;

  /* Standard-deviation model */

  /**
   * Baseline translation std-dev at 1 m / 1 tag, scaled by dist^2 / tagCount in
   * VisionSubsystem.
   */
  public static double LINEAR_STDDEV_BASELINE = 0.02;

  /**
   * Heading std-dev fed to the pose estimator; set huge so vision never moves
   * heading. The gyro is
   * authoritative. 1678 strategy.
   */
  public static final double HEADING_STDDEV_IGNORE = 1e9;

  /**
   * Translation std-dev multiplier applied while actively aiming/aligning (lower
   * = more trust).
   */
  public static final double AIM_LINEAR_STDDEV_MULTIPLIER = 0.6;

  /**
   * Single-tag observations have 180-deg PnP flip ambiguity; distrust their
   * translation heavily.
   */
  public static final double SINGLE_TAG_LINEAR_STDDEV_MULTIPLIER = 5.0;

  /* PhotonVision fallback strategies */

  /** Enables constrained SolvePnP fallback in PhotonVision IO. */
  public static final boolean ENABLE_CONSTRAINED_FALLBACK = true;

  /**
   * Skip constrained fallback when rotating too quickly, since gyro/vision
   * latency mismatch can
   * induce drift.
   */
  public static final double CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC = 0.5;

  /**
   * Trig solve remains usable deeper into rotation than constrained solve, so it
   * gets a separate
   * cutoff.
   */
  public static final double TRIG_MAX_ANGULAR_RATE_RAD_PER_SEC = 1.0;

  /**
   * Weight on heading error in constrained solve. Higher values trust heading
   * more strongly.
   */
  public static final double CONSTRAINED_HEADING_SCALE_FACTOR = 0.2;

  /**
   * Per-camera translation standard-deviation multipliers ordered like
   * APTAG_CAMERA_NAMES.
   */
  public static double[] CAMERA_STDDEV_FACTORS = new double[] { 1.0, 1.0, 1.0, 1.0 };

  /* Coplanar tag handling */

  /**
   * Tags are considered coplanar (same Hub face) when the angle between their
   * outward Z-axis
   * normals is within this many degrees. Coplanar multi-tag observations have the
   * same rotational
   * ambiguity as single-tag and receive the single-tag std-dev penalty.
   */
  public static final double COPLANAR_ANGLE_THRESHOLD_DEG = 15.0;

  /**
   * If true, coplanar multi-tag observations receive the single-tag distrust
   * multiplier. Kept
   * configurable because some camera layouts observed over-strict behavior when
   * always penalizing
   * coplanar sets.
   */
  public static final boolean APPLY_COPLANAR_PENALTY = Boolean
      .parseBoolean(System.getProperty("vision.applyCoplanarPenalty", "false"));

  /**
   * Static PhotonPoseEstimator strategy chain used only when dynamic strategy
   * selection is disabled
   * or {@code vision.photon.strategyOrder} is set. Supported names include:
   * MULTI_TAG_PNP_ON_COPROCESSOR, CONSTRAINED_SOLVEPNP,
   * PNP_DISTANCE_TRIG_SOLVE,
   * LOWEST_AMBIGUITY.
   */
  public static final String PHOTON_POSE_STRATEGY_ORDER = System.getProperty(
      "vision.photon.strategyOrder",
      "CONSTRAINED_SOLVEPNP,MULTI_TAG_PNP_ON_COPROCESSOR,PNP_DISTANCE_TRIG_SOLVE,LOWEST_AMBIGUITY");
}
