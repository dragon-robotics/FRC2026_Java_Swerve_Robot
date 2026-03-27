package frc.robot.util.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.math.util.Units;
import java.util.List;

public final class ShooterConstants {

  public static final int SHOOTER_HOOD_MOTOR_ID = 13;
  public static final int SHOOTER_KICKER_MOTOR_ID = 14;
  public static final int SHOOTER_LEAD_MOTOR_ID = 15;
  public static final int SHOOTER_FOLLOW_MOTOR_ID = 16;

  public static final int SHOOTER_CANCODER_ID = 2;

  public static final double SHOOTER_KICKER_DUTY_CYCLE = 1.0;
  public static final double SHOOTER_KICKER_VOLTAGE = 12.0;
  public static final double SHOOTER_KICKER_RPM = 3000.0;
  public static final double SHOOTER_LEAD_DUTY_CYCLE = 1.0;
  public static final double SHOOTER_LEAD_VOLTAGE = 12.0;
  public static final double SHOOTER_LEAD_RPM = 2500.0;

  public static enum ShooterHoodSettings {
    HOME(0),
    MIDDLE_CLOSE(0.75),
    MIDDLE(1.5),
    MIDDLE_FAR(2.25),
    FAR(3);

    private final double setpoint;

    ShooterHoodSettings(double setpoint) {
      this.setpoint = setpoint;
    }

    public double getSetting() {
      return setpoint;
    }
  }

  public record ShooterSetpoint(double shooterRPM, ShooterHoodSettings hoodSetting) {}

  public record ShooterZone(
      double minDistanceMeters, double maxDistanceMeters, ShooterSetpoint setpoint) {
    public boolean contains(double distanceMeters) {
      return distanceMeters >= minDistanceMeters && distanceMeters < maxDistanceMeters;
    }
  }

  // Example 3 zones (TODO: Tune these values in real world testing)
  public static final List<ShooterZone> SHOOTER_ZONES =
      List.of(
          new ShooterZone(
              0.0, Units.feetToMeters(2), new ShooterSetpoint(2500, ShooterHoodSettings.HOME)),
          new ShooterZone(
              Units.feetToMeters(2),
              Units.feetToMeters(3),
              new ShooterSetpoint(2550.0, ShooterHoodSettings.HOME)),
          new ShooterZone(
              Units.feetToMeters(3),
              Units.feetToMeters(4),
              new ShooterSetpoint(2600.0, ShooterHoodSettings.HOME)),
          new ShooterZone(
              Units.feetToMeters(4),
              Units.feetToMeters(5),
              new ShooterSetpoint(2650.0, ShooterHoodSettings.HOME)),
          new ShooterZone(
              Units.feetToMeters(5),
              Units.feetToMeters(6),
              new ShooterSetpoint(2700.0, ShooterHoodSettings.HOME)),
          new ShooterZone(
              Units.feetToMeters(6),
              Units.feetToMeters(7),
              new ShooterSetpoint(2750.0, ShooterHoodSettings.HOME)),
          new ShooterZone(
              Units.feetToMeters(7),
              Units.feetToMeters(8),
              new ShooterSetpoint(2800.0, ShooterHoodSettings.HOME)),
          new ShooterZone(
              Units.feetToMeters(8),
              Units.feetToMeters(9),
              new ShooterSetpoint(2850.0, ShooterHoodSettings.HOME)),
          new ShooterZone(
              Units.feetToMeters(9),
              Units.feetToMeters(10),
              new ShooterSetpoint(2900.0, ShooterHoodSettings.HOME)),
          new ShooterZone(
              Units.feetToMeters(10),
              Units.feetToMeters(11),
              new ShooterSetpoint(2950.0, ShooterHoodSettings.HOME)),
          new ShooterZone(
              Units.feetToMeters(11),
              Units.feetToMeters(12),
              new ShooterSetpoint(3000.0, ShooterHoodSettings.MIDDLE_CLOSE)),
          new ShooterZone(
              Units.feetToMeters(12),
              Units.feetToMeters(13),
              new ShooterSetpoint(3050.0, ShooterHoodSettings.MIDDLE_CLOSE)),
          new ShooterZone(
              Units.feetToMeters(13),
              Units.feetToMeters(14),
              new ShooterSetpoint(3100.0, ShooterHoodSettings.MIDDLE_CLOSE)),
          new ShooterZone(
              Units.feetToMeters(14),
              Units.feetToMeters(15),
              new ShooterSetpoint(3150.0, ShooterHoodSettings.MIDDLE_CLOSE)),
          new ShooterZone(
              Units.feetToMeters(15),
              Units.feetToMeters(16),
              new ShooterSetpoint(3200.0, ShooterHoodSettings.MIDDLE_CLOSE)),
          new ShooterZone(
              Units.feetToMeters(16),
              Units.feetToMeters(17),
              new ShooterSetpoint(3250.0, ShooterHoodSettings.MIDDLE_CLOSE)),
          new ShooterZone(
              Units.feetToMeters(17),
              Units.feetToMeters(18),
              new ShooterSetpoint(3300.0, ShooterHoodSettings.MIDDLE)),
          new ShooterZone(
              Units.feetToMeters(18),
              Units.feetToMeters(19),
              new ShooterSetpoint(3350.0, ShooterHoodSettings.MIDDLE)),
          new ShooterZone(
              Units.feetToMeters(19),
              Units.feetToMeters(20),
              new ShooterSetpoint(3400.0, ShooterHoodSettings.MIDDLE)),
          new ShooterZone(
              Units.feetToMeters(20),
              Double.POSITIVE_INFINITY,
              new ShooterSetpoint(4500.0, ShooterHoodSettings.FAR)));

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
                  .withStatorCurrentLimit(Amps.of(100))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLowerLimit(Amps.of(40))
                  .withSupplyCurrentLowerTime(Seconds.of(1)))
          .withVoltage(new VoltageConfigs().withPeakForwardVoltage(12).withPeakReverseVoltage(-12))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(
              new Slot0Configs()
                  .withKP(6)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKS(4.325)
                  .withKV(0.013)
                  .withKA(0.0));

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
                  .withStatorCurrentLimit(Amps.of(100))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLowerLimit(Amps.of(40))
                  .withSupplyCurrentLowerTime(Seconds.of(1)))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(Volts.of(12))
                  .withPeakReverseVoltage(Volts.of(-12)))
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
                  .withSupplyCurrentLowerTime(Seconds.of(1)))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(Volts.of(12))
                  .withPeakReverseVoltage(Volts.of(-12)))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive));

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
                  .withStatorCurrentLimit(Amps.of(30))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(20)))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(Volts.of(10))
                  .withPeakReverseVoltage(Volts.of(-10)))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(
              new Slot0Configs()
                  .withKP(8.0)
                  .withKI(0.0)
                  .withKD(0.1)
                  .withKS(0.0)
                  .withKV(0.0)
                  .withKA(0.0)
                  .withKG(0.4)
                  .withGravityType(GravityTypeValue.Elevator_Static)
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
