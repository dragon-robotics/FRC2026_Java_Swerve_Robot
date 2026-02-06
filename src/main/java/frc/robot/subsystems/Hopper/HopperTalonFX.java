package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.HopperConstants.*;
public class HopperTalonFX implements HopperIO {
    
    private final TalonFX rollerMotor;
    private final TalonFX expandingMotor;

    private final MotionMagicVelocityTorqueCurrentFOC
        rollerMotionMagicVelocityTorqueCurrentFOC;
    private final MotionMagicExpoTorqueCurrentFOC 
        expandingMotorMotionMagicTorqueCurrentFOC;

    public HopperTalonFX () {
        rollerMotor = new TalonFX(ROLLER_MOTOR_ID);
        expandingMotor = new TalonFX(EXPANDING_MOTOR_ID);

        TalonFXConfiguration rollerConfig = 
            new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(ROLLER_STATOR_CURRENT_LIMIT)
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(ROLLER_SUPPLY_CURRENT_LIMIT)
                )
                .withVoltage(
                    new VoltageConfigs()
                        .withPeakForwardVoltage(ROLLER_MAX_VOLTAGE)
                        .withPeakForwardVoltage(-ROLLER_MAX_VOLTAGE)
                ).withOpenLoopRamps(
                    new OpenLoopRampsConfigs()
                        .withDutyCycleOpenLoopRampPeriod(ROLLER_RAMP_RATE)
                        .withTorqueOpenLoopRampPeriod(ROLLER_RAMP_RATE)
                        .withVoltageOpenLoopRampPeriod(ROLLER_RAMP_RATE)
                ).withClosedLoopRamps(
                    new ClosedLoopRampsConfigs()
                        .withDutyCycleClosedLoopRampPeriod(ROLLER_RAMP_RATE)
                        .withTorqueClosedLoopRampPeriod(ROLLER_RAMP_RATE)
                        .withVoltageClosedLoopRampPeriod(ROLLER_RAMP_RATE)   
                ).withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

                TalonFXConfiguration expandingConfig = 
                    new TalonFXConfiguration()
                        .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                .withStatorCurrentLimitEnable(true)
                                .withStatorCurrentLimit(EXPANDING_STATOR_CURRENT_LIMIT)
                                .withSupplyCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(EXPANDING_SUPPLY_CURRENT_LIMIT)
                        )
                        .withVoltage(
                            new VoltageConfigs()
                            .withPeakForwardVoltage(EXPANDING_MAX_VOLTAGE)
                            .withPeakForwardVoltage(-EXPANDING_MAX_VOLTAGE)
                        ).withOpenLoopRamps(
                            new OpenLoopRampsConfigs()
                            .withDutyCycleOpenLoopRampPeriod(EXPANDING_RAMP_RATE)
                            .withTorqueOpenLoopRampPeriod(EXPANDING_RAMP_RATE)
                            .withVoltageOpenLoopRampPeriod(EXPANDING_RAMP_RATE)
                        ).withClosedLoopRamps(
                            new ClosedLoopRampsConfigs()
                            .withDutyCycleClosedLoopRampPeriod(EXPANDING_RAMP_RATE)
                            .withTorqueClosedLoopRampPeriod(EXPANDING_RAMP_RATE)
                            .withVoltageClosedLoopRampPeriod(EXPANDING_RAMP_RATE)
                        ).withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

                // roller motor PID configs
                Slot0Configs rollerConfigs = rollerConfig.Slot0; //@TODO: to be tested and tuned 
                rollerConfigs.kS = 0.25; 
                rollerConfigs.kV = 0.12; 
                rollerConfigs.kA = 0.01; 
                rollerConfigs.kP = 0.11; 
                rollerConfigs.kI = 0.0; 
                rollerConfigs.kD = 0.0; 
                // expanding motor PID configs
                Slot1Configs expandingConfigs = expandingConfig.Slot1; //@TODO: to be tested and tuned 
                expandingConfigs.kS = 0.25; 
                expandingConfigs.kV = 0.12; 
                expandingConfigs.kA = 0.01; 
                expandingConfigs.kP = 0.11; 
                expandingConfigs.kI = 0.0; 
                expandingConfigs.kD = 0.0; 

                MotionMagicConfigs hopperMotionMagicConfigs = rollerConfig.MotionMagic;

                hopperMotionMagicConfigs.MotionMagicAcceleration = 200;

                hopperMotionMagicConfigs.MotionMagicJerk = 2000;

                rollerMotor.getConfigurator().apply(rollerConfig);
                expandingMotor.getConfigurator().apply(expandingConfig);
                rollerMotionMagicVelocityTorqueCurrentFOC = new MotionMagicVelocityTorqueCurrentFOC(0);
                expandingMotorMotionMagicTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC (0);

    }

    @Override 
    public void expandHopper(double setpoint) {
        expandingMotor.setControl(
            expandingMotorMotionMagicTorqueCurrentFOC.withPosition(setpoint)
        );
    }
    
}
