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
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
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

public final class IntakeConstants {

  public static final int INTAKE_ROLLER_MOTOR_ID = 11;
  public static final int INTAKE_ARM_MOTOR_ID = 12;
  public static final int INTAKE_ARM_CANCODER_ID = 0;
  public static final int INTAKE_ROLLER_CANCODER_ID = 1;

  public static final double INTAKE_ARM_LENGTH_METERS = Units.inchesToMeters(18);
  public static final double INTAKE_ARM_MASS_KG = Units.lbsToKilograms(10);
  public static final double INTAKE_ARM_GEAR_RATIO = 36;
  public static final double INTAKE_MIN_ANGLE_RADIANS = Units.degreesToRadians(0);
  public static final double INTAKE_MAX_ANGLE_RADIANS = Units.degreesToRadians(90);
  public static final double INTAKE_STARTING_ANGLE_RADIANS = INTAKE_MAX_ANGLE_RADIANS;

  public static final double INTAKE_ROLLER_DUTY_CYCLE = 0.5;
  public static final double INTAKE_ROLLER_VOLTAGE = 5.0;
  public static final double INTAKE_ROLLER_RPM = 4000.0;
  public static final double OUTTAKE_ROLLER_DUTY_CYCLE = -0.5;
  public static final double OUTTAKE_ROLLER_VOLTAGE = -5.0;
  public static final double OUTTAKE_ROLLER_RPM = -4000.0;

  public static final double INTAKE_ARM_STOWED_POSITION = -0.25;
  public static final double INTAKE_ARM_STOWED_ANGLE_DEG = Units.degreesToRadians(90);
  public static final double INTAKE_ARM_DEPLOYED_POSITION = 0.0;
  public static final double INTAKE_ARM_DEPLOYED_ANGLE_DEG = Units.degreesToRadians(0);
  public static final double INTAKE_ARM_POSITION_TOLERANCE = 5.0;

  public static final TalonFXConfiguration INTAKE_ROLLER_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(40)))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(Volts.of(12))
                  .withPeakReverseVoltage(Volts.of(-12)))
          .withOpenLoopRamps(
              new OpenLoopRampsConfigs()
                  .withDutyCycleOpenLoopRampPeriod(Seconds.of(0.1))
                  .withTorqueOpenLoopRampPeriod(Seconds.of(0.1))
                  .withVoltageOpenLoopRampPeriod(Seconds.of(0.1)))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0)
                  .withKA(0)
                  .withKG(0)
                  .withKP(0)
                  .withKI(0)
                  .withKD(0))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(0)
                  .withMotionMagicAcceleration(100)
                  .withMotionMagicJerk(100)
                  .withMotionMagicExpo_kV(0.1)
                  .withMotionMagicExpo_kA(0.1));

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

  public static final TalonFXConfiguration INTAKE_ARM_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(80))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLowerLimit(40)
                  .withSupplyCurrentLowerTime(1))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(Volts.of(12))
                  .withPeakReverseVoltage(Volts.of(-12)))
          .withOpenLoopRamps(
              new OpenLoopRampsConfigs()
                  .withDutyCycleOpenLoopRampPeriod(Seconds.of(0.1))
                  .withTorqueOpenLoopRampPeriod(Seconds.of(0.1))
                  .withVoltageOpenLoopRampPeriod(Seconds.of(0.1)))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0.0)
                  .withKA(0.0)
                  .withKG(0)
                  .withKP(0)
                  .withKI(0)
                  .withKD(0)
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
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                  .withSensorToMechanismRatio(1)
                  .withRotorToSensorRatio(INTAKE_ARM_GEAR_RATIO)
                  .withFeedbackRotorOffset(0));

  public static final CANcoderConfiguration INTAKE_ARM_CANCODER_CONFIG =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withAbsoluteSensorDiscontinuityPoint(0.5)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  .withMagnetOffset(-0.123));

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
