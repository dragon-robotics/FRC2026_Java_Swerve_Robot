package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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

    public static class Hub {
      public static final double BLUE_HUB_CENTER_X =
          (APTAG_FIELD_LAYOUT.getTagPose(20).get().getX()
                  + APTAG_FIELD_LAYOUT.getTagPose(26).get().getX())
              / 2;
      public static final Translation2d BLUE_HUB_CENTER_POSE =
          new Translation2d(BLUE_HUB_CENTER_X, APTAG_FIELD_LAYOUT.getTagPose(20).get().getY());
      public static final double RED_HUB_CENTER_X =
          (APTAG_FIELD_LAYOUT.getTagPose(4).get().getX()
                  + APTAG_FIELD_LAYOUT.getTagPose(10).get().getX())
              / 2;
      public static final Translation2d RED_HUB_CENTER_POSE =
          new Translation2d(RED_HUB_CENTER_X, APTAG_FIELD_LAYOUT.getTagPose(4).get().getY());
    }
  }

  public static class VisionConstants {
    public static final String[] APTAG_CAMERA_NAMES = {
      "LEFT_CAM",
      "RIGHT_CAM",
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
                Units.inchesToMeters(7.3885)),
            new Rotation3d(0, Units.degreesToRadians(-20), 0));

    // Main Apriltag alignment cam mounted facing forward, half a meter forward of
    // center, half a meter up from center.
    public static final Transform3d APTAG_ALIGN_RIGHT_CAM_POS =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(9.749),
                Units.inchesToMeters(-4.910),
                Units.inchesToMeters(7.3885)),
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
  }

  public static class SwerveConstants {
    // General constants for swerve drive //
    public static final double STEER_KP = 100;
    public static final double STEER_KI = 0;
    public static final double STEER_KD = 0.5;
    public static final double STEER_KS = 0.1;
    public static final double STEER_KV = 1.59;
    public static final double STEER_KA = 0;

    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0;
    public static final double DRIVE_KS = 0;
    public static final double DRIVE_KV = 0.124;
    public static final double DRIVE_KA = 0;

    public static final double HEADING_KP = 8;
    public static final double HEADING_KI = 0;
    public static final double HEADING_KD = 0.3;
    public static final double HEADING_TOLERANCE = 0.01;

    public static final double ANGLE_GEAR_RATIO = 12.8;
    public static final double DRIVE_GEAR_RATIO = 6.12;
    public static final double PULSE_PER_ROTATION = 1;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
    public static final double MAX_SPEED_FEET_PER_SECOND = 18.2; // 18.2 feet per second
    public static final double MAX_SPEED_METERS_PER_SECOND =
        Units.feetToMeters(MAX_SPEED_FEET_PER_SECOND); // 18.2 feet
    // per
    // second

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592;
    public static final double CHASSIS_MASS = ROBOT_MASS;
    public static final Translation3d CHASSIS_CG = new Translation3d(0, 0, Units.inchesToMeters(8));
    public static final double LOOP_TIME = 0.13;

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10;

    public static final double SWERVE_DEADBAND = 0.1;

    // SWERVE MODULE ODOMETRY STANDARD DEVIATIONS //
    // public static final Matrix<N3, N1> ODOMETRY_STD = VecBuilder.fill(0.1, 0.1,
    // Units.degreesToRadians(5));
    public static final Matrix<N3, N1> ODOMETRY_STD =
        VecBuilder.fill(
            Units.inchesToMeters(0.5), Units.inchesToMeters(0.5), Units.degreesToRadians(3));
  }

  public static class IntakeSubsystemConstants {

    public static final int INTAKE_ROLLER_MOTOR_ID = 11;
    public static final int INTAKE_ARM_MOTOR_ID = 12;
    public static final int INTAKE_ARM_CANCODER_ID = 0;

    // Intake arm physical properties
    public static final double INTAKE_ARM_LENGTH_METERS = Units.inchesToMeters(18);
    public static final double INTAKE_ARM_MASS_KG = Units.lbsToKilograms(10);
    public static final double INTAKE_ARM_GEAR_RATIO = 20;
    public static final double INTAKE_MIN_ANGLE_RADIANS = Units.degreesToRadians(0);
    public static final double INTAKE_MAX_ANGLE_RADIANS = Units.degreesToRadians(90);
    public static final double INTAKE_STARTING_ANGLE_RADIANS = INTAKE_MAX_ANGLE_RADIANS;

    /* Intake roller motor configurations */
    public static final TalonFXConfiguration INTAKE_ROLLER_TALONFX_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40)))
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(Volts.of(10))
                    .withPeakReverseVoltage(Volts.of(-10)))
            .withOpenLoopRamps(
                new OpenLoopRampsConfigs()
                    .withDutyCycleOpenLoopRampPeriod(Seconds.of(0.1))
                    .withTorqueOpenLoopRampPeriod(Seconds.of(0.1))
                    .withVoltageOpenLoopRampPeriod(Seconds.of(0.1)))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(
                new Slot0Configs()
                    .withKS(0)
                    .withKV(0.4)
                    .withKA(0.17)
                    .withKG(0.62)
                    .withKP(0.81)
                    .withKI(0)
                    .withKD(0.23))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(60)
                    .withMotionMagicAcceleration(100)
                    .withMotionMagicJerk(100));

    public static final SparkBaseConfig INTAKE_ROLLER_SPARKMAX_CONFIG =
        new SparkMaxConfig()
            .apply(
                new SparkMaxConfig()
                    .voltageCompensation(10)
                    .smartCurrentLimit(40, 20)
                    .secondaryCurrentLimit(60)
                    .openLoopRampRate(0.1)
                    .idleMode(IdleMode.kCoast))
            .apply(
                new ClosedLoopConfig()
                    .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                    .outputRange(-1, 1)
                    .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0)
                    .apply(
                        new MAXMotionConfig()
                            .cruiseVelocity(4000, ClosedLoopSlot.kSlot0)
                            .maxAcceleration(8000, ClosedLoopSlot.kSlot0)
                            .allowedProfileError(40, ClosedLoopSlot.kSlot0)));

    /* Desired controls for intake roller */
    public static final double INTAKE_ROLLER_DUTY_CYCLE = 0.5;
    public static final double INTAKE_ROLLER_VOLTAGE = 5.0;
    public static final double INTAKE_ROLLER_RPM = 4000.0;
    public static final double OUTTAKE_ROLLER_DUTY_CYCLE = -0.5;
    public static final double OUTTAKE_ROLLER_VOLTAGE = -5.0;
    public static final double OUTTAKE_ROLLER_RPM = -4000.0;

    /* Intake arm motor configurations */
    public static final TalonFXConfiguration INTAKE_ARM_TALONFX_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(60))
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40)))
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(Volts.of(10))
                    .withPeakReverseVoltage(Volts.of(-10)))
            .withOpenLoopRamps(
                new OpenLoopRampsConfigs()
                    .withDutyCycleOpenLoopRampPeriod(Seconds.of(0.2))
                    .withTorqueOpenLoopRampPeriod(Seconds.of(0.2))
                    .withVoltageOpenLoopRampPeriod(Seconds.of(0.2)))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(
                new Slot0Configs()
                    .withKS(0)
                    .withKV(0.4)
                    .withKA(0.17)
                    .withKG(0.62)
                    .withKP(0.81)
                    .withKI(0)
                    .withKD(0.23))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(100)
                    .withMotionMagicCruiseVelocity(0) // Unlimited cruise velocity
                    .withMotionMagicExpo_kV(0.12)
                    .withMotionMagicExpo_kA(0.1))
            .withFeedback(
                new FeedbackConfigs()
                    .withFusedCANcoder(new CoreCANcoder(INTAKE_ARM_CANCODER_ID))
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withSensorToMechanismRatio(1)
                    .withRotorToSensorRatio(INTAKE_ARM_GEAR_RATIO)
                    .withFeedbackRotorOffset(0));

    public static final CANcoderConfiguration INTAKE_ARM_CANCODER_CONFIG =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    // choose one; common is signed ±0.5 rotations
                    .withAbsoluteSensorDiscontinuityPoint(0.5)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    // set this to your calibrated zero (rotations)
                    .withMagnetOffset(0.0));

    public static final SparkBaseConfig INTAKE_ARM_SPARKMAX_CONFIG =
        new SparkMaxConfig()
            .apply(
                new SparkMaxConfig()
                    .voltageCompensation(10)
                    .smartCurrentLimit(40, 20)
                    .secondaryCurrentLimit(60)
                    .openLoopRampRate(0.2)
                    .idleMode(IdleMode.kBrake))
            .apply(new AbsoluteEncoderConfig().zeroOffset(0).inverted(false))
            .apply(
                new ClosedLoopConfig()
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0)
                    .outputRange(-1, 1)
                    .apply(
                        new FeedForwardConfig()
                            .kS(0.0, ClosedLoopSlot.kSlot0)
                            .kV(0.5, ClosedLoopSlot.kSlot0)
                            .kA(0.06, ClosedLoopSlot.kSlot0)
                            .kG(0.39, ClosedLoopSlot.kSlot0))
                    .apply(
                        new MAXMotionConfig()
                            .cruiseVelocity(4000, ClosedLoopSlot.kSlot0)
                            .maxAcceleration(8000, ClosedLoopSlot.kSlot0)
                            .allowedProfileError(5, ClosedLoopSlot.kSlot0)
                            .positionMode(
                                MAXMotionPositionMode.kMAXMotionTrapezoidal,
                                ClosedLoopSlot.kSlot0)));

    public static final AbsoluteEncoderConfig INTAKE_ARM_ENCODER_CONFIG =
        new AbsoluteEncoderConfig().zeroOffset(0).inverted(false).zeroCentered(true);

    /* Desired controls for intake arm */
    public static final double INTAKE_ARM_STOWED_POSITION = 0.25; // Setpoint
    public static final double INTAKE_ARM_STOWED_ANGLE_DEG = Units.degreesToRadians(90); // Setpoint
    public static final double INTAKE_ARM_DEPLOYED_POSITION = 0.0; // Setpoint
    public static final double INTAKE_ARM_DEPLOYED_ANGLE_DEG =
        Units.degreesToRadians(0); // Setpoint
    public static final double INTAKE_ARM_POSITION_TOLERANCE = 5.0; // Setpoint
  }

  public static class OperatorConstants {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final int OPERATOR_BUTTON_PORT = 2;
    public static final int TEST_PORT = 3;
  }

  public static class JoystickConstants {
    // Joystick Analog Axis/Stick //
    public static final int STICK_LEFT_X = 0;
    public static final int STICK_LEFT_Y = 1;
    public static final int TRIGGER_LEFT = 2;
    public static final int TRIGGER_RIGHT = 3;
    public static final int STICK_RIGHT_X = 4;
    public static final int STICK_RIGHT_Y = 5;

    // Joystick Buttons //
    public static final int BTN_A = 1;
    public static final int BTN_B = 2;
    public static final int BTN_X = 3;
    public static final int BTN_Y = 4;
    public static final int BUMPER_LEFT = 5;
    public static final int BUMPER_RIGHT = 6;
    public static final int BTN_BACK = 7;
    public static final int BTN_START = 8;
    public static final int BTN_STICK_LEFT = 9;
    public static final int BTN_STICK_RIGHT = 10;
  }

  public static class ShooterConstants {
    public static boolean IS_SHOOTING = false; 
    public static final int FORWARD_MOTOR_ID = 3;
    public static final int INVERSE_MOTOR_ID = 4; 
    public static final double SHOOTER_STATOR_CURRENT_LIMIT = 20.0;
    public static final double SHOOTER_SUPPLY_CURRENT_LIMIT = 40.0; 
    public static final double SHOOTER_MAX_VOLTAGE = 10.0; 
    public static final double SHOOTER_RAMP_RATE = 0.5;
    public static final double SHOOTER_RPM = 4000.0; // placeholder value
  }
}