package frc.robot.util.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
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
import java.util.List;

public final class ShooterConstants {

  public static final int SHOOTER_HOOD_MOTOR_ID = 13;
  public static final int SHOOTER_KICKER_MOTOR_ID = 14;
  public static final int SHOOTER_LEAD_MOTOR_ID = 15;
  public static final int SHOOTER_FOLLOW_MOTOR_ID = 16;

  public static final int SHOOTER_CANCODER_ID = 2;

  public static final double SHOOTER_KICKER_RPM = 3000.0;

  public record ShooterSetpoint(double shooterRPM, double hoodAngleDeg) {}

  public record ShooterZone(
      double minDistanceMeters, double maxDistanceMeters, ShooterSetpoint setpoint) {
    public boolean contains(double distanceMeters) {
      return distanceMeters >= minDistanceMeters && distanceMeters < maxDistanceMeters;
    }
  }

  // Example 3 zones (TODO: Tune these values in real world testing)
  public static final List<ShooterZone> SHOOTER_ZONES =
      List.of(
          new ShooterZone(0.0, 2.0, new ShooterSetpoint(3000.0, 0.12)), // close
          new ShooterZone(2.0, 4.0, new ShooterSetpoint(3800.0, 0.28)), // mid
          new ShooterZone(4.0, Double.POSITIVE_INFINITY, new ShooterSetpoint(4500.0, 0.42)) // far
          );

  public static ShooterSetpoint getSetpointForDistance(double distanceMeters) {
    for (ShooterZone zone : SHOOTER_ZONES) {
      if (zone.contains(distanceMeters)) {
        return zone.setpoint();
      }
    }
    return SHOOTER_ZONES.get(SHOOTER_ZONES.size() - 1).setpoint();
  }

  public static final TalonFXConfiguration SHOOTER_LEAD_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(80))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLowerLimit(20)
                  .withSupplyCurrentLowerTime(1))
          .withVoltage(new VoltageConfigs().withPeakForwardVoltage(10).withPeakReverseVoltage(-10))
          .withOpenLoopRamps(
              new OpenLoopRampsConfigs()
                  .withDutyCycleOpenLoopRampPeriod(Seconds.of(0.1))
                  .withTorqueOpenLoopRampPeriod(Seconds.of(0.1))
                  .withVoltageOpenLoopRampPeriod(Seconds.of(0.1)))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(
              new Slot0Configs()
                  .withKS(4.325)
                  .withKV(0.013)
                  .withKA(0.0)
                  .withKP(1.5)
                  .withKI(0.0)
                  .withKD(0.0));

  public static final SparkBaseConfig SHOOTER_LEAD_SPARKMAX_CONFIG =
      new SparkMaxConfig()
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

  public static final TalonFXConfiguration SHOOTER_FOLLOW_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(80))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLowerLimit(20)
                  .withSupplyCurrentLowerTime(1))
          .withVoltage(new VoltageConfigs().withPeakForwardVoltage(10).withPeakReverseVoltage(-10))
          .withOpenLoopRamps(
              new OpenLoopRampsConfigs()
                  .withDutyCycleOpenLoopRampPeriod(Seconds.of(0.1))
                  .withTorqueOpenLoopRampPeriod(Seconds.of(0.1))
                  .withVoltageOpenLoopRampPeriod(Seconds.of(0.1)))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

  public static final SparkBaseConfig SHOOTER_FOLLOW_SPARKMAX_CONFIG =
      new SparkMaxConfig()
          .apply(
              new SparkMaxConfig()
                  .voltageCompensation(10)
                  .smartCurrentLimit(60, 20)
                  .secondaryCurrentLimit(80)
                  .openLoopRampRate(0.1)
                  .idleMode(IdleMode.kCoast)
                  .follow(SHOOTER_LEAD_MOTOR_ID));

  public static final TalonFXConfiguration SHOOTER_KICKER_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(80))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLowerLimit(Amps.of(40))
                  .withSupplyCurrentLowerTime(1))
          .withVoltage(new VoltageConfigs().withPeakForwardVoltage(10).withPeakReverseVoltage(-10))
          .withOpenLoopRamps(
              new OpenLoopRampsConfigs()
                  .withDutyCycleOpenLoopRampPeriod(Seconds.of(0.1))
                  .withTorqueOpenLoopRampPeriod(Seconds.of(0.1))
                  .withVoltageOpenLoopRampPeriod(Seconds.of(0.1)))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(
              new Slot0Configs()
                  .withKS(14.4)
                  .withKV(0.11)
                  .withKA(0.0)
                  .withKP(1.5)
                  .withKI(0.0)
                  .withKD(0.0));

  public static final SparkBaseConfig SHOOTER_KICKER_SPARKMAX_CONFIG =
      new SparkMaxConfig()
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

  public static final TalonFXConfiguration SHOOTER_HOOD_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(20))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(10)))
          .withVoltage(new VoltageConfigs().withPeakForwardVoltage(10).withPeakReverseVoltage(-10))
          .withOpenLoopRamps(
              new OpenLoopRampsConfigs()
                  .withDutyCycleOpenLoopRampPeriod(Seconds.of(0.1))
                  .withTorqueOpenLoopRampPeriod(Seconds.of(0.1))
                  .withVoltageOpenLoopRampPeriod(Seconds.of(0.1)))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(
              new Slot0Configs()
                  .withKS(0.0)
                  .withKV(0.0)
                  .withKA(0.0)
                  .withKG(0.4)
                  .withGravityType(GravityTypeValue.Elevator_Static)
                  .withKP(8.0)
                  .withKI(0.0)
                  .withKD(0.1)
                  .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));

  public static final CANcoderConfiguration SHOOTER_HOOD_CANCODER_CONFIG =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  // choose one; common is signed ±0.5 rotations
                  .withAbsoluteSensorDiscontinuityPoint(0.5)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  // set this to your calibrated zero (rotations)
                  .withMagnetOffset(0.0));

  public static final SparkBaseConfig SHOOTER_HOOD_SPARKMAX_CONFIG =
      new SparkMaxConfig()
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
