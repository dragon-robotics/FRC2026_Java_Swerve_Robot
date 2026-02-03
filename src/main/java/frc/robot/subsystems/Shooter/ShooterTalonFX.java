package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
public class ShooterTalonFX implements ShooterIO {

        private final TalonFX forwardMotor;
        private final TalonFX inverseMotor; 
        
        public ShooterTalonFX() {
            forwardMotor = new TalonFX(Constants.ShooterConstants.FORWARD_MOTOR_ID);
            inverseMotor = new TalonFX(Constants.ShooterConstants.INVERSE_MOTOR_ID);
    
            TalonFXConfiguration shooterConfig =
                new TalonFXConfiguration()
                    .withCurrentLimits(
                        new CurrentLimitsConfigs()
                            .withStatorCurrentLimitEnable(true)
                            .withStatorCurrentLimit(Amps.of(Constants.ShooterConstants.SHOOTER_SUPPLY_CURRENT_LIMIT))
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(Amps.of(Constants.ShooterConstants.SHOOTER_SUPPLY_CURRENT_LIMIT)
                            )
                )
                .withVoltage(
                    new VoltageConfigs()
                        .withPeakForwardVoltage(Volts.of(Constants.ShooterConstants.SHOOTER_MAX_VOLTAGE))
                        .withPeakReverseVoltage(Volts.of(-Constants.ShooterConstants.SHOOTER_MAX_VOLTAGE))
                ).withOpenLoopRamps(
                    new OpenLoopRampsConfigs()
                        .withDutyCycleOpenLoopRampPeriod(Seconds.of(Constants.ShooterConstants.SHOOTER_RAMP_RATE))
                        .withTorqueOpenLoopRampPeriod(Seconds.of(Constants.ShooterConstants.SHOOTER_RAMP_RATE))
                        .withVoltageOpenLoopRampPeriod(Seconds.of(Constants.ShooterConstants.SHOOTER_RAMP_RATE))
                ).withClosedLoopRamps(
                    new ClosedLoopRampsConfigs()
                        .withDutyCycleClosedLoopRampPeriod(Seconds.of(Constants.ShooterConstants.SHOOTER_RAMP_RATE))
                        .withTorqueClosedLoopRampPeriod(Seconds.of(Constants.ShooterConstants.SHOOTER_RAMP_RATE))
                        .withVoltageClosedLoopRampPeriod(Seconds.of(Constants.ShooterConstants.SHOOTER_RAMP_RATE))       
                ).withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

                forwardMotor.getConfigurator().apply(shooterConfig);
                inverseMotor.getConfigurator().apply(shooterConfig);
    }

    public void runShooter(double rpm) {
        Constants.ShooterConstants.IS_SHOOTING = true;
        forwardMotor.setControl(new VelocityVoltage(rpm));
        inverseMotor.setControl(new VelocityVoltage(-rpm));
    }
    public void stopShooter() {
        Constants.ShooterConstants.IS_SHOOTING = false; 
        runShooter(0.0);
    }
    // returns the average velocity of both motors
    public double getShooterSpeed() {
        double forward = forwardMotor.getVelocity().getValueAsDouble();
        double inverse = inverseMotor.getVelocity().getValueAsDouble();
        
        return (forward + inverse) / 2;    
    }

    @Override
    public void updateInputs(ShooterIOInputs shooterInputs) {
        
        shooterInputs.setForwardMotorConnected(forwardMotor.isConnected());
        shooterInputs.setInverseMotorConnected(inverseMotor.isConnected());
        
        // grab forward motor data and set it 
        shooterInputs.setForwardMotorVelocity(forwardMotor.getVelocity().getValueAsDouble());
        shooterInputs.setForwardMotorTemprature(forwardMotor.getDeviceTemp().getValueAsDouble());
        shooterInputs.setForwardMotorVoltage(forwardMotor.getMotorVoltage().getValueAsDouble());
        shooterInputs.setForwardMotorCurrent(forwardMotor.getStatorCurrent().getValueAsDouble());
        // grab inverse motor data and set it 
        shooterInputs.setInverseMotorVelocity(inverseMotor.getVelocity().getValueAsDouble());
        shooterInputs.setInverseMotorCurrent(inverseMotor.getStatorCurrent().getValueAsDouble());
        shooterInputs.setInverseMotorTemprature(inverseMotor.getDeviceTemp().getValueAsDouble());
        shooterInputs.setInverseMotorVoltage(inverseMotor.getMotorVoltage().getValueAsDouble());
    }
}
