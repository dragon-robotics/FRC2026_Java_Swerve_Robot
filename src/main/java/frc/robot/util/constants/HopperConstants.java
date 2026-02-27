package frc.robot.util.constants;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class HopperConstants {
  public static final int ROLLER_MOTOR_ID = 17;
  public static final double ROLLER_STATOR_CURRENT_LIMIT = 20.0;
  public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double ROLLER_MAX_VOLTAGE = 10.0;
  public static final double ROLLER_RAMP_RATE = 0.5;
  public static final double ROLLER_RPM = 1000.0;
  public static final double ROLLER_REVERSE_RPM = -1000.0;

  public static final TalonFXConfiguration HOPPER_ROLLER_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(ROLLER_STATOR_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(ROLLER_SUPPLY_CURRENT_LIMIT))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(ROLLER_MAX_VOLTAGE)
                  .withPeakReverseVoltage(-ROLLER_MAX_VOLTAGE))
          .withOpenLoopRamps(
              new OpenLoopRampsConfigs()
                  .withDutyCycleOpenLoopRampPeriod(ROLLER_RAMP_RATE)
                  .withTorqueOpenLoopRampPeriod(ROLLER_RAMP_RATE)
                  .withVoltageOpenLoopRampPeriod(ROLLER_RAMP_RATE))
          .withClosedLoopRamps(
              new ClosedLoopRampsConfigs()
                  .withDutyCycleClosedLoopRampPeriod(ROLLER_RAMP_RATE)
                  .withTorqueClosedLoopRampPeriod(ROLLER_RAMP_RATE)
                  .withVoltageClosedLoopRampPeriod(ROLLER_RAMP_RATE))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withMotionMagic(
              new MotionMagicConfigs().withMotionMagicAcceleration(200).withMotionMagicJerk(2000))
          .withSlot0(
              new Slot0Configs()
                  .withKS(0.0)
                  .withKV(0.0)
                  .withKA(0.0)
                  .withKP(0.0)
                  .withKI(0.0)
                  .withKD(0.0));
}
