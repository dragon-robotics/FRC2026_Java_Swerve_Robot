package frc.robot.util.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
  public static final String[] APTAG_CAMERA_NAMES = {
    "AprilTagPoseEstCameraF",
    "AprilTagPoseEstCameraR",
    "AprilTagPoseEstCameraB",
    "AprilTagPoseEstCameraL"
  };

  public static final Transform3d APTAG_POSE_EST_CAM_F_POS =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-11.4), Units.inchesToMeters(-7.7), Units.inchesToMeters(21.25)),
          new Rotation3d(0, Units.degreesToRadians(-15), 0));

  public static final Transform3d APTAG_POSE_EST_CAM_R_POS =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-8.64),
              Units.inchesToMeters(-13.35),
              Units.inchesToMeters(15.9)),
          new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-90)));

  public static final Transform3d APTAG_POSE_EST_CAM_B_POS =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-9.42), Units.inchesToMeters(12.5), Units.inchesToMeters(20.84)),
          new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(180)));

  public static final Transform3d APTAG_POSE_EST_CAM_L_POS =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-8.64),
              Units.inchesToMeters(13.36),
              Units.inchesToMeters(15.93)),
          new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(90)));

  public static final Transform3d[] APTAG_POSE_EST_CAM_POSITIONS = {
    APTAG_POSE_EST_CAM_F_POS,
    APTAG_POSE_EST_CAM_R_POS,
    APTAG_POSE_EST_CAM_B_POS,
    APTAG_POSE_EST_CAM_L_POS
  };

  public static final Matrix<N3, N1> SINGLE_TAG_STDDEV = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> MULTI_TAG_STDDEV = VecBuilder.fill(0.5, 0.5, 1);
  public static final Matrix<N3, N1> DEFAULT_TAG_STDDEV = VecBuilder.fill(0.9, 0.9, 0.9);

  public static double MAX_AMBIGUITY = 0.1;
  public static double MAX_Z_ERROR = Units.inchesToMeters(1.5);

  public static double LINEAR_STDDEV_BASELINE = Units.inchesToMeters(1);
  public static double ANGULAR_STDDEV_BASELINE = Units.degreesToRadians(5);

  public static final double CAMERA_FOV_HORIZONTAL_DEGREES = 73.0;
  public static final double CAMERA_ASPECT_RATIO_WIDTH = 4.0;
  public static final double CAMERA_ASPECT_RATIO_HEIGHT = 3.0;

  public static final double CAMERA_FOV_VERTICAL_DEGREES =
      calculateVerticalFOV(
          CAMERA_FOV_HORIZONTAL_DEGREES, CAMERA_ASPECT_RATIO_WIDTH, CAMERA_ASPECT_RATIO_HEIGHT);

  private static double calculateVerticalFOV(
      double horizontalFOV, double aspectRatioWidth, double aspectRatioHeight) {
    double horizontalFOV_rad = Math.toRadians(horizontalFOV);
    double tan_hFOV_half = Math.tan(horizontalFOV_rad / 2.0);
    double tan_vFOV_half = tan_hFOV_half * (aspectRatioHeight / aspectRatioWidth);
    double vFOV_half_rad = Math.atan(tan_vFOV_half);
    return Math.toDegrees(vFOV_half_rad * 2.0);
  }

  public static double[] CAMERA_STDDEV_FACTORS = new double[] {1.0, 1.0};

  public static double LINEAR_STDDEV_MEGATAG2_FACTOR = 0.5;
  public static double ANGULAR_STDDEV_MEGATAG2_ANGLE_FACTOR = Double.POSITIVE_INFINITY;
}
