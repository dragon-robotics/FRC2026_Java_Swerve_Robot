package frc.robot.util.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * Swerve drivetrain gains, geometry, physical model values, and estimator uncertainty constants.
 *
 * <p>Distances are meters unless a constant name explicitly says inches or feet. Angular values are
 * radians unless a constant name explicitly says degrees.
 */
public final class SwerveConstants {

  /* Steering closed-loop gains */

  public static final double STEER_KP = 100;
  public static final double STEER_KI = 0;
  public static final double STEER_KD = 0.5;
  public static final double STEER_KS = 0.1;
  public static final double STEER_KV = 1.59;
  public static final double STEER_KA = 0;

  /* Drive closed-loop gains */

  public static final double DRIVE_KP = 0.1;
  public static final double DRIVE_KI = 0;
  public static final double DRIVE_KD = 0;
  public static final double DRIVE_KS = 0;
  public static final double DRIVE_KV = 0.124;
  public static final double DRIVE_KA = 0;

  /* Heading controller */

  public static final double HEADING_KP = 8;
  public static final double HEADING_KI = 0;
  public static final double HEADING_KD = 0.3;
  public static final double HEADING_TOLERANCE = Units.degreesToRadians(5);

  /* Module geometry and limits */

  public static final double ANGLE_GEAR_RATIO = 12.8;
  public static final double DRIVE_GEAR_RATIO = 6.12;
  public static final double PULSE_PER_ROTATION = 1;
  public static final double WHEEL_DIAMETER_INCHES = 4.0;
  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
  public static final double MAX_SPEED_FEET_PER_SECOND = 18.2;
  public static final double MAX_SPEED_METERS_PER_SECOND =
      Units.feetToMeters(MAX_SPEED_FEET_PER_SECOND);

  /* Robot dimensions */

  public static final double ROBOT_WIDTH_WITH_BUMPERS_METERS = Units.inchesToMeters(33);
  public static final double ROBOT_WIDTH_METERS = Units.inchesToMeters(27);
  public static final double ROBOT_LENGTH_WITH_BUMPERS_METERS = Units.inchesToMeters(33);
  public static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(27);
  public static final double ROBOT_CENTER_TO_WIDTH_METERS = ROBOT_WIDTH_METERS / 2;
  public static final double ROBOT_CENTER_TO_WIDTH_WITH_BUMPERS_METERS =
      ROBOT_WIDTH_WITH_BUMPERS_METERS / 2;
  public static final double ROBOT_CENTER_TO_LENGTH_METERS = ROBOT_LENGTH_METERS / 2;
  public static final double ROBOT_CENTER_TO_LENGTH_WITH_BUMPERS_METERS =
      ROBOT_LENGTH_WITH_BUMPERS_METERS / 2;
  public static final double ROBOT_CENTER_TO_CORNER_METERS =
      Math.hypot(ROBOT_LENGTH_METERS / 2, ROBOT_WIDTH_METERS / 2);
  public static final double ROBOT_CENTER_TO_CORNER_WITH_BUMPERS_METERS =
      Math.hypot(ROBOT_LENGTH_WITH_BUMPERS_METERS / 2, ROBOT_WIDTH_WITH_BUMPERS_METERS / 2);

  /* Simulation and physics */

  /** Robot mass in kilograms. */
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592;

  public static final double CHASSIS_MASS = ROBOT_MASS;

  /** Chassis center of gravity in the robot frame, in meters. */
  public static final Translation3d CHASSIS_CG = new Translation3d(0, 0, Units.inchesToMeters(8));

  /** Nominal loop time in seconds used by drivetrain simulation helpers. */
  public static final double LOOP_TIME = 0.13;

  /** Time in seconds to hold wheel-lock alignment. */
  public static final double WHEEL_LOCK_TIME = 10;

  /* Driver control */

  public static final double SWERVE_DEADBAND = 0.1;

  /* Odometry estimator uncertainty */

  public static final double ODOMETRY_X_STD_METERS = Units.inchesToMeters(2);
  public static final double ODOMETRY_Y_STD_METERS = Units.inchesToMeters(2);
  public static final double ODOMETRY_HEADING_STD_RADIANS = Units.degreesToRadians(1);

  /** Odometry standard deviations ordered as x meters, y meters, and heading radians. */
  public static final Matrix<N3, N1> ODOMETRY_STD =
      VecBuilder.fill(ODOMETRY_X_STD_METERS, ODOMETRY_Y_STD_METERS, ODOMETRY_HEADING_STD_RADIANS);
}
