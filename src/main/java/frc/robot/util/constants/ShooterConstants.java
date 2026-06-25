package frc.robot.util.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
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

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/**
 * Shooter hardware IDs, motor-controller configuration, and scoring setpoints.
 *
 * <p>
 * Distances are stored in meters. Flywheel setpoints are RPM. Hood positions
 * are mechanism
 * rotations from the hood encoder.
 */
public final class ShooterConstants {

  /* Hardware IDs */

  public static final int SHOOTER_HOOD_MOTOR_ID = 13;
  public static final int SHOOTER_KICKER_MOTOR_ID = 14;
  public static final int SHOOTER_LEAD_MOTOR_ID = 15;
  public static final int SHOOTER_FOLLOW_MOTOR_ID = 16;

  public static final int SHOOTER_CANCODER_ID = 2;

  /* Flywheel */

  public static final Voltage SHOOTER_VOLTAGE = Volts.of(12.0);
  public static final Current SHOOTER_STATOR_CURRENT_LIMIT = Amps.of(60.0);
  public static final Current SHOOTER_SUPPLY_CURRENT_LIMIT = Amps.of(40.0);
  public static final Current SHOOTER_SUPPLY_CURRENT_LOWER_LIMIT = Amps.of(20.0);
  public static final Time SHOOTER_SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.25);
  public static final double SHOOTER_P = 8.0;
  public static final double SHOOTER_S = 4.325;
  public static final double SHOOTER_V = 0.013;
  public static final double SHOOTER_DUTY_CYCLE = 1.0;
  public static final double SHOOTER_RPM = 2500.0;
  // public static final double SHOOTER_PREP_RPM = 1200.0;
  public static final double SHOOTER_PREP_RPM = 0;
  public static final double SHOOTER_READY_TOLERANCE_RPM = 60.0;
  public static final double SHOOTER_STOPPED_TOLERANCE_RPM = 0.5;

  /* Kicker */

  public static final Voltage SHOOTER_KICKER_VOLTAGE = Volts.of(12.0);
  public static final Current SHOOTER_KICKER_STATOR_CURRENT_LIMIT = Amps.of(80.0);
  public static final Current SHOOTER_KICKER_SUPPLY_CURRENT_LIMIT = Amps.of(40.0);
  public static final Current SHOOTER_KICKER_SUPPLY_CURRENT_LOWER_LIMIT = Amps.of(30.0);
  public static final Time SHOOTER_KICKER_SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.25);
  public static final double SHOOTER_KICKER_DUTY_CYCLE = 1.0;
  // public static final Voltage SHOOTER_KICKER_PREP_VOLTAGE = Volts.of(6.0);
  public static final Voltage SHOOTER_KICKER_PREP_VOLTAGE = Volts.of(0.0);
  public static final double SHOOTER_KICKER_STOP_PERCENT_OUTPUT = 0.5;
  public static final double SHOOTER_KICKER_STOP_DELAY_SECONDS = 1.0;

  /* Hood */

  public static final Voltage SHOOTER_HOOD_VOLTAGE = Volts.of(10.0);
  public static final Current SHOOTER_HOOD_STATOR_CURRENT_LIMIT = Amps.of(25.0);
  public static final Current SHOOTER_HOOD_SUPPLY_CURRENT_LIMIT = Amps.of(15.0);
  public static final double SHOOTER_HOOD_P = 8.0;
  public static final double SHOOTER_HOOD_D = 0.1;
  public static final double SHOOTER_HOOD_G = 0.4;
  public static final double SHOOTER_HOOD_DEFAULT_SETTING = 0.0;
  public static final double SHOOTER_HOOD_READY_TOLERANCE_ROTATIONS = 0.125;

  /**
   * Shooter flywheel RPM and hood position in mechanism rotations for one target
   * distance.
   */
  public record ShooterSetpoint(double shooterRPM, double hoodAngle) {
  }

  /** Interpolating table for shooter RPM based on distance in meters. */
  public static final InterpolatingDoubleTreeMap SHOOTER_RPM_MAP = new InterpolatingDoubleTreeMap();

  /**
   * Interpolating table for hood position in mechanism rotations based on
   * distance in meters.
   */
  public static final InterpolatingDoubleTreeMap SHOOTER_HOOD_MAP = new InterpolatingDoubleTreeMap();

  static {
    // Distances are authored in feet for tuning readability, then stored as meters.
    SHOOTER_RPM_MAP.put(Units.feetToMeters(5), 2450.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(6), 2500.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(7), 2550.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(8), 2700.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(9), 2800.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(10), 2850.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(11), 2900.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(12), 3000.0);

    // Hood positions are mechanism rotations.
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(5), 0.00);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(6), 0.00);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(7), 0.00);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(8), 0.00);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(9), 0.00);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(10), 0.75);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(11), 0.75);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(12), 1.25);
  }

  /**
   * Returns interpolated flywheel RPM and hood rotations for a target distance in
   * meters.
   */
  public static ShooterSetpoint getSetpointForDistance(double distanceMeters) {
    return new ShooterSetpoint(
        SHOOTER_RPM_MAP.get(distanceMeters), SHOOTER_HOOD_MAP.get(distanceMeters));
  }

  public static final TalonFXConfiguration SHOOTER_LEAD_TALONFX_CONFIG = new TalonFXConfiguration()
      .withCurrentLimits(
          new CurrentLimitsConfigs()
              .withStatorCurrentLimitEnable(true)
              .withStatorCurrentLimit(SHOOTER_STATOR_CURRENT_LIMIT)
              .withSupplyCurrentLimitEnable(true)
              .withSupplyCurrentLimit(SHOOTER_SUPPLY_CURRENT_LIMIT)
              .withSupplyCurrentLowerLimit(SHOOTER_SUPPLY_CURRENT_LOWER_LIMIT)
              .withSupplyCurrentLowerTime(SHOOTER_SUPPLY_CURRENT_LOWER_TIME))
      .withVoltage(
          new VoltageConfigs()
              .withPeakForwardVoltage(SHOOTER_VOLTAGE)
              .withPeakReverseVoltage(SHOOTER_VOLTAGE.unaryMinus()))
      .withMotorOutput(
          new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Coast)
              .withInverted(InvertedValue.Clockwise_Positive))
      .withSlot0(
          new Slot0Configs()
              .withKP(SHOOTER_P)
              .withKI(0.0)
              .withKD(0.0)
              .withKS(SHOOTER_S)
              .withKV(SHOOTER_V)
              .withKA(0.0));

  public static final SparkBaseConfig SHOOTER_LEAD_SPARKMAX_CONFIG = new SparkMaxConfig()
      .apply(
          new SparkMaxConfig()
              .voltageCompensation(10)
              .smartCurrentLimit(60, 20)
              .secondaryCurrentLimit(80)
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

  public static final TalonFXConfiguration SHOOTER_FOLLOW_TALONFX_CONFIG = new TalonFXConfiguration()
      .withCurrentLimits(
          new CurrentLimitsConfigs()
              .withStatorCurrentLimitEnable(true)
              .withStatorCurrentLimit(SHOOTER_STATOR_CURRENT_LIMIT)
              .withSupplyCurrentLimitEnable(true)
              .withSupplyCurrentLimit(SHOOTER_SUPPLY_CURRENT_LIMIT)
              .withSupplyCurrentLowerLimit(SHOOTER_SUPPLY_CURRENT_LOWER_LIMIT)
              .withSupplyCurrentLowerTime(SHOOTER_SUPPLY_CURRENT_LOWER_TIME))
      .withVoltage(
          new VoltageConfigs()
              .withPeakForwardVoltage(SHOOTER_VOLTAGE)
              .withPeakReverseVoltage(SHOOTER_VOLTAGE.unaryMinus()))
      .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

  public static final SparkBaseConfig SHOOTER_FOLLOW_SPARKMAX_CONFIG = new SparkMaxConfig()
      .apply(
          new SparkMaxConfig()
              .voltageCompensation(10)
              .smartCurrentLimit(60, 20)
              .secondaryCurrentLimit(80)
              .openLoopRampRate(0.1)
              .idleMode(IdleMode.kCoast)
              .follow(SHOOTER_LEAD_MOTOR_ID));

  public static final TalonFXConfiguration SHOOTER_KICKER_TALONFX_CONFIG = new TalonFXConfiguration()
      .withCurrentLimits(
          new CurrentLimitsConfigs()
              .withStatorCurrentLimitEnable(true)
              .withStatorCurrentLimit(SHOOTER_KICKER_STATOR_CURRENT_LIMIT)
              .withSupplyCurrentLimitEnable(true)
              .withSupplyCurrentLimit(SHOOTER_KICKER_SUPPLY_CURRENT_LIMIT)
              .withSupplyCurrentLowerLimit(SHOOTER_KICKER_SUPPLY_CURRENT_LOWER_LIMIT)
              .withSupplyCurrentLowerTime(SHOOTER_KICKER_SUPPLY_CURRENT_LOWER_TIME))
      .withVoltage(
          new VoltageConfigs()
              .withPeakForwardVoltage(SHOOTER_KICKER_VOLTAGE)
              .withPeakReverseVoltage(SHOOTER_KICKER_VOLTAGE.unaryMinus()))
      .withMotorOutput(
          new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Coast)
              .withInverted(InvertedValue.Clockwise_Positive));

  public static final SparkBaseConfig SHOOTER_KICKER_SPARKMAX_CONFIG = new SparkMaxConfig()
      .apply(
          new SparkMaxConfig()
              .voltageCompensation(10)
              .smartCurrentLimit(20, 10)
              .secondaryCurrentLimit(40)
              .openLoopRampRate(0.1)
              .idleMode(IdleMode.kCoast))
      .apply(
          new ClosedLoopConfig()
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .outputRange(-1, 1)
              .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0)
              .apply(
                  new MAXMotionConfig()
                      .cruiseVelocity(4000, ClosedLoopSlot.kSlot0)
                      .maxAcceleration(8000, ClosedLoopSlot.kSlot0)
                      .allowedProfileError(40, ClosedLoopSlot.kSlot0)));

  public static final TalonFXConfiguration SHOOTER_HOOD_TALONFX_CONFIG = new TalonFXConfiguration()
      .withCurrentLimits(
          new CurrentLimitsConfigs()
              .withStatorCurrentLimitEnable(true)
              .withStatorCurrentLimit(SHOOTER_HOOD_STATOR_CURRENT_LIMIT)
              .withSupplyCurrentLimitEnable(true)
              .withSupplyCurrentLimit(SHOOTER_HOOD_SUPPLY_CURRENT_LIMIT))
      .withVoltage(
          new VoltageConfigs()
              .withPeakForwardVoltage(SHOOTER_HOOD_VOLTAGE)
              .withPeakReverseVoltage(SHOOTER_HOOD_VOLTAGE.unaryMinus()))
      .withMotorOutput(
          new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Brake)
              .withInverted(InvertedValue.Clockwise_Positive))
      .withSlot0(
          new Slot0Configs()
              .withKP(SHOOTER_HOOD_P)
              .withKI(0.0)
              .withKD(SHOOTER_HOOD_D)
              .withKS(0.0)
              .withKV(0.0)
              .withKA(0.0)
              .withKG(SHOOTER_HOOD_G)
              .withGravityType(GravityTypeValue.Elevator_Static)
              .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));

  public static final CANcoderConfiguration SHOOTER_HOOD_CANCODER_CONFIG = new CANcoderConfiguration()
      .withMagnetSensor(
          new MagnetSensorConfigs()
              .withAbsoluteSensorDiscontinuityPoint(0.5)
              .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
              .withMagnetOffset(0.0));

  public static final SparkBaseConfig SHOOTER_HOOD_SPARKMAX_CONFIG = new SparkMaxConfig()
      .apply(
          new SparkMaxConfig()
              .voltageCompensation(10)
              .smartCurrentLimit(15, 10)
              .secondaryCurrentLimit(30)
              .openLoopRampRate(0.1)
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
                      .kV(0.0, ClosedLoopSlot.kSlot0)
                      .kA(0.0, ClosedLoopSlot.kSlot0)
                      .kG(0.0, ClosedLoopSlot.kSlot0))
              .apply(
                  new MAXMotionConfig()
                      .cruiseVelocity(4000, ClosedLoopSlot.kSlot0)
                      .maxAcceleration(8000, ClosedLoopSlot.kSlot0)
                      .allowedProfileError(5, ClosedLoopSlot.kSlot0)
                      .positionMode(
                          MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0)));
}
