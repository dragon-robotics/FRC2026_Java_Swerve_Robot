package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.IOException;
import java.nio.file.Path;

public class Constants {
  /** General robot constants */
  public static final class GeneralConstants {

    // Robot mode
    public static final RobotMode CURRENT_MODE =
        RobotBase.isReal() ? RobotMode.COMP : RobotMode.SIM;

    public static enum RobotMode {
      /** Running on test mode */
      TEST,
      /** Running on competition mode */
      COMP,
      /** Running on simulation mode */
      SIM
    }

    public static boolean disableHAL = false;
  }

  public static class FieldConstants {

    // The different layouts of the AprilTags on the field
    public static final AprilTagFieldLayout APTAG_FIELD_LAYOUT;

    // Static initializer block
    static {
      // Load default layout - this does NOT throw IOException
      // It might return null if the resource is missing, though kDefaultField should
      // be safe.
      // Construct paths for welded layouts
      Path defaultPath =
          Path.of(
              Filesystem.getDeployDirectory().getPath(),
              "apriltags",
              "welded",
              "2026-rebuilt-welded.json");
      AprilTagFieldLayout defaultLayout = null;
      try {
        defaultLayout = new AprilTagFieldLayout(defaultPath);
      } catch (IOException e) {
        System.err.println("!!! CRITICAL: Failed to load default AprilTag field resource!");
        DriverStation.reportError(
            "CRITICAL: Failed to load default AprilTag field resource: " + e.getMessage(), true);

        // If loading from file fails, we will use the static kDefaultField layout
        // as a fallback.
        defaultLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
      }

      // Assign the layout to be the default layout
      APTAG_FIELD_LAYOUT = defaultLayout;
    }

    public static final double FIELD_LENGTH = APTAG_FIELD_LAYOUT.getFieldLength();
    public static final double FIELD_WIDTH = APTAG_FIELD_LAYOUT.getFieldWidth();

    // Positions to line to for shooting //
  }

  public static class VisionConstants {
    public static final String[] APTAG_CAMERA_NAMES = {
      "AprilTagAlignLeftCamera",
      "AprilTagAlignRightCamera",
      "AprilTagPoseEstCameraFL",
      "AprilTagPoseEstCameraFR",
      "AprilTagPoseEstCameraBL",
      "AprilTagPoseEstCameraBR"
    };

    // Main Apriltag alignment cam mounted facing forward, half a meter forward of
    // center, half a meter up from center.
    public static final Transform3d APTAG_ALIGN_LEFT_CAM_POS =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(9.249),
                Units.inchesToMeters(4.910),
                Units.inchesToMeters(8.3885)),
            new Rotation3d(0, Units.degreesToRadians(-20), 0));

    // Main Apriltag alignment cam mounted facing forward, half a meter forward of
    // center, half a meter up from center.
    public static final Transform3d APTAG_ALIGN_RIGHT_CAM_POS =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(9.249),
                Units.inchesToMeters(-4.910),
                Units.inchesToMeters(8.3885)),
            new Rotation3d(0, Units.degreesToRadians(-20), 0));

    // Front-Left Camera: Mounted at front-left corner, pointing outward at 30
    // degrees
    public static final Transform3d APTAG_POSE_EST_CAM_FL_POS =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(17.125),
                Units.inchesToMeters(17.125),
                Units.inchesToMeters(6.825)),
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(45)));

    // Front-Right Camera: Mounted at front-right corner, pointing outward at -30
    // degrees
    public static final Transform3d APTAG_POSE_EST_CAM_FR_POS =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(17.125),
                Units.inchesToMeters(-17.125),
                Units.inchesToMeters(6.825)),
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-45)));

    // Back-Left Camera: Mounted at back-left corner, pointing outward at 135
    // degrees
    public static final Transform3d APTAG_POSE_EST_CAM_BL_POS =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-17.125),
                Units.inchesToMeters(17.125),
                Units.inchesToMeters(6.825)),
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(135)));

    // Back-Right Camera: Mounted at back-right corner, pointing outward at -135
    // degrees
    public static final Transform3d APTAG_POSE_EST_CAM_BR_POS =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-17.125),
                Units.inchesToMeters(-17.125),
                Units.inchesToMeters(6.825)),
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-135)));

    public static final Transform3d[] APTAG_POSE_EST_CAM_POSITIONS = {
      APTAG_POSE_EST_CAM_FL_POS,
      APTAG_POSE_EST_CAM_FR_POS,
      APTAG_POSE_EST_CAM_BL_POS,
      APTAG_POSE_EST_CAM_BR_POS
    };

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> SINGLE_TAG_STDDEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STDDEV = VecBuilder.fill(0.5, 0.5, 1);
    public static final Matrix<N3, N1> DEFAULT_TAG_STDDEV = VecBuilder.fill(0.9, 0.9, 0.9);

    // Basic filtering thresholds
    public static double MAX_AMBIGUITY = 0.1;
    public static double MAX_Z_ERROR = Units.inchesToMeters(0.5);

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double LINEAR_STDDEV_BASELINE = Units.inchesToMeters(1); // Inches to Meters
    public static double ANGULAR_STDDEV_BASELINE = Units.degreesToRadians(5); // Degrees to Radians

    // Known values
    public static final double CAMERA_FOV_HORIZONTAL_DEGREES = 73.0; // Your known horizontal FOV
    public static final double CAMERA_ASPECT_RATIO_WIDTH = 4.0;
    public static final double CAMERA_ASPECT_RATIO_HEIGHT = 3.0;

    // Calculated value
    public static final double CAMERA_FOV_VERTICAL_DEGREES =
        calculateVerticalFOV(
            CAMERA_FOV_HORIZONTAL_DEGREES, CAMERA_ASPECT_RATIO_WIDTH, CAMERA_ASPECT_RATIO_HEIGHT);

    /**
     * Calculates the vertical Field of View (FOV) from the horizontal FOV and aspect ratio.
     *
     * @param horizontalFOV Horizontal FOV in degrees.
     * @param aspectRatioWidth Width component of the aspect ratio.
     * @param aspectRatioHeight Height component of the aspect ratio.
     * @return Vertical FOV in degrees.
     */
    private static double calculateVerticalFOV(
        double horizontalFOV, double aspectRatioWidth, double aspectRatioHeight) {
      // Convert horizontal FOV to radians for Math functions
      double horizontalFOV_rad = Math.toRadians(horizontalFOV);

      // Calculate tan(HFOV / 2)
      double tan_hFOV_half = Math.tan(horizontalFOV_rad / 2.0);

      // Calculate tan(VFOV / 2) using the aspect ratio
      double tan_vFOV_half = tan_hFOV_half * (aspectRatioHeight / aspectRatioWidth);

      // Calculate VFOV / 2 in radians
      double vFOV_half_rad = Math.atan(tan_vFOV_half);

      // Calculate VFOV in radians and then convert to degrees
      return Math.toDegrees(vFOV_half_rad * 2.0);
    }

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] CAMERA_STDDEV_FACTORS =
        new double[] {
          1.0, // APTAG_LEFT_CAM
          1.0, // APTAG_RIGHT_CAM
          // 1.0, // APTAG_POSE_EST_CAM_FL_POS
          // 1.0, // APTAG_POSE_EST_CAM_FR_POS
          // 1.0, // APTAG_POSE_EST_CAM_BL_POS
          // 1.0 // APTAG_POSE_EST_CAM_BR_POS
        };

    // Multipliers to apply for MegaTag 2 observations
    public static double LINEAR_STDDEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
    public static double ANGULAR_STDDEV_MEGATAG2_ANGLE_FACTOR = Double.POSITIVE_INFINITY;
    ; // More stable than full 3D
    // solve

    // Vision range and aim PID constants //
    public static final double RANGE_P = 2;
    public static final double RANGE_I = 0;
    public static final double RANGE_D = 0;
    public static final double RANGE_TOLERANCE = 0.02;

    public static final double AIM_P = 2;
    public static final double AIM_I = 0;
    public static final double AIM_D = 0;
    public static final double AIM_TOLERANCE = 0.02;

    public static final double DESIRED_RANGE = 0.25;
    public static final double DESIRED_YAW_RIGHT = 0.03;
    public static final double DESIRED_YAW_LEFT = -0.03;
  }
}
