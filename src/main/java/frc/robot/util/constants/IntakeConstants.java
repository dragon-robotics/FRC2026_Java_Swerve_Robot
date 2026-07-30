package frc.robot.util.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/**
 * Intake hardware IDs, motor-controller configuration, and mechanism setpoints.
 *
 * <p>Arm simulation limits use radians. Arm closed-loop positions use mechanism rotations from the
 * absolute/fused encoder. Roller commands use volts, RPM, or percent output depending on the
 * caller.
 */
public final class IntakeConstants {

  /* Hardware IDs */

  public static final int INTAKE_ROLLER_LEAD_MOTOR_ID = 21;
  public static final int INTAKE_ROLLER_FOLLOW_MOTOR_ID = 20;
  public static final int INTAKE_ARM_MOTOR_ID = 10;
  public static final int INTAKE_ARM_CANCODER_ID = 0;

  /* Arm motion and sim model */

  public static final int INTAKE_ARM_FAST_PID_SLOT = 0;
  public static final int INTAKE_ARM_SLOW_PID_SLOT = 1;
  public static final double INTAKE_ARM_LENGTH_METERS = Units.inchesToMeters(13.370);
  public static final double INTAKE_ARM_MASS_KG = Units.lbsToKilograms(10);
  public static final double INTAKE_ARM_GEAR_RATIO = 36;
  public static final double INTAKE_MIN_ANGLE_RADIANS = Units.degreesToRadians(0);
  public static final double INTAKE_MAX_ANGLE_RADIANS = Units.degreesToRadians(90);
  public static final double INTAKE_STARTING_ANGLE_RADIANS = INTAKE_MIN_ANGLE_RADIANS;

  public static final Voltage INTAKE_ARM_MAX_VOLTAGE = Volts.of(10);
  public static final Current INTAKE_ARM_STATOR_CURRENT_LIMIT = Amps.of(50);
  public static final Current INTAKE_ARM_SUPPLY_CURRENT_LIMIT = Amps.of(30);
  public static final Current INTAKE_ARM_DEPLOY_TENSION_CURRENT = Amps.of(-18.75);
  public static final double INTAKE_ARM_SLOW_P = 8;
  public static final double INTAKE_ARM_FAST_P = 14;
  public static final double INTAKE_ARM_KV = 2.4;
  public static final double INTAKE_ARM_KG = 0.5;
  public static final double INTAKE_ARM_CANCODER_OFFSET = 0.881064453125;

  /** Arm stowed position in mechanism rotations. */
  public static final double INTAKE_ARM_STOWED_POSITION = 0.37;

  /**
   * First juicer position in mechanism rotations.
   *
   * <p>The arm moves here quickly to avoid the hopper wall before squeezing fuel inward.
   */
  public static final double INTAKE_ARM_JUICER_PRE_POSITION = 0.15;

  /** Final juicer squeeze position in mechanism rotations. */
  public static final double INTAKE_ARM_JUICER_FINAL_POSITION = 0.25;

  /** Deployed arm position in mechanism rotations. */
  public static final double INTAKE_ARM_DEPLOYED_POSITION = 0.0;

  /** Allowed arm position error in mechanism rotations. */
  public static final double INTAKE_ARM_POSITION_TOLERANCE = 0.025;

  /* Roller */

  public static final double INTAKE_ROLLER_DUTY_CYCLE = 1.0;
  public static final Voltage INTAKE_ROLLER_VOLTAGE = Volts.of(12);
  public static final double INTAKE_ROLLER_RPM = 6000.0;
  public static final Current INTAKE_ROLLER_TORQUE_CURRENT = Amps.of(80);
  public static final double INTAKE_ROLLER_TORQUE_CURRENT_MAX_DUTY_CYCLE = 0.75;
  public static final double OUTTAKE_ROLLER_DUTY_CYCLE = -1.0;
  public static final Voltage OUTTAKE_ROLLER_VOLTAGE = Volts.of(-12);
  public static final double OUTTAKE_ROLLER_RPM = -6000.0;
  public static final Current OUTTAKE_ROLLER_TORQUE_CURRENT = Amps.of(-80);
  public static final double OUTTAKE_ROLLER_TORQUE_CURRENT_MAX_DUTY_CYCLE = 0.75;

  public static final Current INTAKE_ROLLER_STATOR_CURRENT_LIMIT = Amps.of(80);
  public static final Current INTAKE_ROLLER_SUPPLY_CURRENT_LIMIT = Amps.of(40);
  public static final Current INTAKE_ROLLER_SUPPLY_CURRENT_LOWER_LIMIT = Amps.of(30);
  public static final Time INTAKE_ROLLER_SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.2);

  /* Motor controller configs */

  public static final TalonFXConfiguration INTAKE_ROLLER_LEAD_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(INTAKE_ROLLER_STATOR_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(INTAKE_ROLLER_SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLowerLimit(INTAKE_ROLLER_SUPPLY_CURRENT_LOWER_LIMIT)
                  .withSupplyCurrentLowerTime(INTAKE_ROLLER_SUPPLY_CURRENT_LOWER_TIME))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(INTAKE_ROLLER_VOLTAGE)
                  .withPeakReverseVoltage(OUTTAKE_ROLLER_VOLTAGE))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive));

  public static final SparkBaseConfig INTAKE_ROLLER_LEAD_SPARKMAX_CONFIG =
      new SparkMaxConfig()
          .apply(
              new SparkMaxConfig()
                  .voltageCompensation(10)
                  .smartCurrentLimit(30, 20)
                  .secondaryCurrentLimit(60)
                  .openLoopRampRate(0.15)
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

  public static final TalonFXConfiguration INTAKE_ROLLER_FOLLOW_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(INTAKE_ROLLER_STATOR_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(INTAKE_ROLLER_SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLowerLimit(INTAKE_ROLLER_SUPPLY_CURRENT_LOWER_LIMIT)
                  .withSupplyCurrentLowerTime(INTAKE_ROLLER_SUPPLY_CURRENT_LOWER_TIME))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(INTAKE_ROLLER_VOLTAGE)
                  .withPeakReverseVoltage(OUTTAKE_ROLLER_VOLTAGE))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

  public static final SparkBaseConfig INTAKE_ROLLER_FOLLOW_SPARKMAX_CONFIG =
      new SparkMaxConfig()
          .apply(
              new SparkMaxConfig()
                  .voltageCompensation(10)
                  .smartCurrentLimit(30, 20)
                  .secondaryCurrentLimit(60)
                  .openLoopRampRate(0.15)
                  .idleMode(IdleMode.kCoast)
                  .follow(INTAKE_ROLLER_LEAD_MOTOR_ID));

  public static final TalonFXConfiguration INTAKE_ARM_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(INTAKE_ARM_STATOR_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(INTAKE_ARM_SUPPLY_CURRENT_LIMIT))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(INTAKE_ARM_MAX_VOLTAGE)
                  .withPeakReverseVoltage(INTAKE_ARM_MAX_VOLTAGE.unaryMinus()))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          // Fast deploy profile overcomes the constant-force spring of the extending
          // hopper.
          .withSlot0(
              new Slot0Configs()
                  .withKP(INTAKE_ARM_FAST_P)
                  .withKI(0)
                  .withKD(0)
                  .withKS(0)
                  .withKV(INTAKE_ARM_KV)
                  .withKA(0)
                  .withKG(INTAKE_ARM_KG)
                  .withGravityType(GravityTypeValue.Arm_Cosine)
                  .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
          // Slow retract profile is used by juicer mode to squeeze fuel inward.
          .withSlot1(
              new Slot1Configs()
                  .withKP(INTAKE_ARM_SLOW_P)
                  .withKI(0)
                  .withKD(0)
                  .withKS(0)
                  .withKV(INTAKE_ARM_KV)
                  .withKA(0)
                  .withKG(INTAKE_ARM_KG)
                  .withGravityType(GravityTypeValue.Arm_Cosine)
                  .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(0)
                  .withMotionMagicAcceleration(200)
                  .withMotionMagicJerk(200)
                  .withMotionMagicExpo_kV(2.0)
                  .withMotionMagicExpo_kA(2.0))
          .withFeedback(
              new FeedbackConfigs()
                  .withFusedCANcoder(new CoreCANcoder(INTAKE_ARM_CANCODER_ID))
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                  .withSensorToMechanismRatio(1)
                  .withRotorToSensorRatio(INTAKE_ARM_GEAR_RATIO)
                  .withFeedbackRotorOffset(0));

  public static final CANcoderConfiguration INTAKE_ARM_CANCODER_CONFIG =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withAbsoluteSensorDiscontinuityPoint(0.5)
                  .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                  .withMagnetOffset(INTAKE_ARM_CANCODER_OFFSET));

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

  public static final AbsoluteEncoderConfig INTAKE_ARM_ENCODER_CONFIG =
      new AbsoluteEncoderConfig().zeroOffset(0).inverted(false).zeroCentered(true);
}
