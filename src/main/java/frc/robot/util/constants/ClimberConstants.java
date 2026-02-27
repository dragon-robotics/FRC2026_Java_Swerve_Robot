package frc.robot.util.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public final class ClimberConstants {
  public static final int LEAD_MOTOR_ID = 7;
  public static final int FOLLOWER_MOTOR_ID = 8;
  public static final int CLIMBER_CANCODER_ID = 1;
  public static final double CLIMBER_GEAR_RATIO = 10;
  public static final double CLIMBER_STATOR_CURRENT_LIMIT = 40.0;
  public static final double CLIMBER_SUPPLY_CURRENT_LIMIT = 60.0;
  public static final double CLIMBER_MAX_VOLTAGE = 10.0;
  public static final double CLIMBER_RAMP_RATE = 0.5;
  public static final double CLIMBER_HOME_SETPOINT = 0.0;
  public static final double CLIMBER_RETRACTED_SETPOINT = 25.0;
  public static final double CLIMBER_L1_SETPOINT = 50.0;
  public static final double POSITION_TOLERANCE = 0.5;

  public static final double CLIMBER_LENGTH_METERS = Units.inchesToMeters(24);
  public static final double CLIMBER_MASS_KG = Units.lbsToKilograms(12);
  public static final double CLIMBER_HOME_ANGLE = Units.degreesToRadians(0);
  public static final double CLIMBER_MIN_ANGLE_RADIANS = Units.degreesToRadians(0);
  public static final double CLIMBER_MAX_ANGLE_RADIANS = Units.degreesToRadians(45);
  public static final double CLIMBER_STARTING_ANGLE_RADIANS = CLIMBER_HOME_ANGLE;

  public static final TalonFXConfiguration CLIMBER_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(CLIMBER_STATOR_CURRENT_LIMIT))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(CLIMBER_SUPPLY_CURRENT_LIMIT)))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(Volts.of(CLIMBER_MAX_VOLTAGE))
                  .withPeakReverseVoltage(Volts.of(-CLIMBER_MAX_VOLTAGE)))
          .withOpenLoopRamps(
              new OpenLoopRampsConfigs()
                  .withDutyCycleOpenLoopRampPeriod(Seconds.of(CLIMBER_RAMP_RATE))
                  .withTorqueOpenLoopRampPeriod(Seconds.of(CLIMBER_RAMP_RATE))
                  .withVoltageOpenLoopRampPeriod(Seconds.of(CLIMBER_RAMP_RATE)))
          .withClosedLoopRamps(
              new ClosedLoopRampsConfigs()
                  .withDutyCycleClosedLoopRampPeriod(Seconds.of(CLIMBER_RAMP_RATE))
                  .withTorqueClosedLoopRampPeriod(Seconds.of(CLIMBER_RAMP_RATE))
                  .withVoltageClosedLoopRampPeriod(Seconds.of(CLIMBER_RAMP_RATE)))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(
              new Slot0Configs()
                  .withKS(0.0)
                  .withKV(0.0)
                  .withKA(0.0)
                  .withKP(0.0)
                  .withKI(0.0)
                  .withKD(0.0))
          .withMotionMagic(
              new MotionMagicConfigs().withMotionMagicAcceleration(100).withMotionMagicJerk(100));
}
