package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeSubsystemConstants.*;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
    
    private final TalonFX intakeRollerMotor;
    private final TalonFX intakeArmMotor;
    
    public IntakeIOTalonFX() {

        /* Instantiate the motors */
        intakeRollerMotor = new TalonFX(INTAKE_ROLLER_MOTOR_ID);
        intakeArmMotor = new TalonFX(INTAKE_ARM_MOTOR_ID);

        /* Configure the motors */
        /* Roller motor configuration */
        TalonFXConfiguration rollerConfig =
            new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(Amps.of(INTAKE_ROLLER_STATOR_CURRENT_LIMIT))
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(INTAKE_ROLLER_SUPPLY_CURRENT_LIMIT))
                )
                .withVoltage(
                    new VoltageConfigs()
                        .withPeakForwardVoltage(Volts.of(INTAKE_ROLLER_MAX_VOLTAGE))
                        .withPeakReverseVoltage(Volts.of(-INTAKE_ROLLER_MAX_VOLTAGE)))
                .withOpenLoopRamps(
                    new OpenLoopRampsConfigs()
                        .withDutyCycleOpenLoopRampPeriod(Seconds.of(INTAKE_ROLLER_RAMP_RATE))
                        .withTorqueOpenLoopRampPeriod(Seconds.of(INTAKE_ROLLER_RAMP_RATE))
                        .withVoltageOpenLoopRampPeriod(Seconds.of(INTAKE_ROLLER_RAMP_RATE)))
                .withClosedLoopRamps(
                    new ClosedLoopRampsConfigs()
                        .withDutyCycleClosedLoopRampPeriod(Seconds.of(INTAKE_ROLLER_RAMP_RATE))
                        .withTorqueClosedLoopRampPeriod(Seconds.of(INTAKE_ROLLER_RAMP_RATE))
                        .withVoltageClosedLoopRampPeriod(Seconds.of(INTAKE_ROLLER_RAMP_RATE)))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(INTAKE_ROLLER_NEUTRAL_MODE));

        /* Arm motor configuration */
        TalonFXConfiguration armConfig =
            new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(Amps.of(INTAKE_ARM_STATOR_CURRENT_LIMIT))
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(INTAKE_ARM_SUPPLY_CURRENT_LIMIT))
                )
                .withVoltage(
                    new VoltageConfigs()
                        .withPeakForwardVoltage(Volts.of(INTAKE_ARM_MAX_VOLTAGE))
                        .withPeakReverseVoltage(Volts.of(-INTAKE_ARM_MAX_VOLTAGE)))
                .withOpenLoopRamps(
                    new OpenLoopRampsConfigs()
                        .withDutyCycleOpenLoopRampPeriod(Seconds.of(INTAKE_ARM_RAMP_RATE))
                        .withTorqueOpenLoopRampPeriod(Seconds.of(INTAKE_ARM_RAMP_RATE))
                        .withVoltageOpenLoopRampPeriod(Seconds.of(INTAKE_ARM_RAMP_RATE)))
                .withClosedLoopRamps(
                    new ClosedLoopRampsConfigs()
                        .withDutyCycleClosedLoopRampPeriod(Seconds.of(INTAKE_ARM_RAMP_RATE))
                        .withTorqueClosedLoopRampPeriod(Seconds.of(INTAKE_ARM_RAMP_RATE))
                        .withVoltageClosedLoopRampPeriod(Seconds.of(INTAKE_ARM_RAMP_RATE)))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(INTAKE_ARM_NEUTRAL_MODE));

        intakeRollerMotor.getConfigurator().apply(rollerConfig);
        intakeArmMotor.getConfigurator().apply(armConfig);
    }

    
}
