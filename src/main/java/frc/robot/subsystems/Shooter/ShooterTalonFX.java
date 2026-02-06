package frc.robot.subsystems.shooter;

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
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterTalonFX implements ShooterIO {

    private final TalonFX forwardMotor;
    private final TalonFX inverseMotor; 
    
    private final MotionMagicVelocityTorqueCurrentFOC
        shooterMotionMagicVelocityTorqueCurrentFOCRequest;

    public ShooterTalonFX() {
        forwardMotor = new TalonFX(FORWARD_MOTOR_ID);
        inverseMotor = new TalonFX(INVERSE_MOTOR_ID);

        TalonFXConfiguration shooterConfig =
            new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(Amps.of(SHOOTER_SUPPLY_CURRENT_LIMIT))
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(SHOOTER_SUPPLY_CURRENT_LIMIT)
                        )
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(Volts.of(SHOOTER_MAX_VOLTAGE))
                    .withPeakReverseVoltage(Volts.of(-SHOOTER_MAX_VOLTAGE))
            ).withOpenLoopRamps(
                new OpenLoopRampsConfigs()
                    .withDutyCycleOpenLoopRampPeriod(Seconds.of(SHOOTER_RAMP_RATE))
                    .withTorqueOpenLoopRampPeriod(Seconds.of(SHOOTER_RAMP_RATE))
                    .withVoltageOpenLoopRampPeriod(Seconds.of(SHOOTER_RAMP_RATE))
            ).withClosedLoopRamps(
                new ClosedLoopRampsConfigs()
                    .withDutyCycleClosedLoopRampPeriod(Seconds.of(SHOOTER_RAMP_RATE))
                    .withTorqueClosedLoopRampPeriod(Seconds.of(SHOOTER_RAMP_RATE))
                    .withVoltageClosedLoopRampPeriod(Seconds.of(SHOOTER_RAMP_RATE))       
            ).withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        /* shooter mechanism PID and velocity congiurations */
        Slot0Configs shooterConfigs = shooterConfig.Slot0; //@TODO: NEEDS TO BE TESTED AND TUNED
        shooterConfigs.kS = 0.25; 
        shooterConfigs.kV = 0.12; 
        shooterConfigs.kA = 0.01; 
        shooterConfigs.kP = 0.11; 
        shooterConfigs.kI = 0.0; 
        shooterConfigs.kD = 0.0;
        // Motion Magic configuration and Velocity settings
        MotionMagicConfigs shooterMotionMagicConfigs = shooterConfig.MotionMagic;
        // Acceleration is 400 rp/s
        shooterMotionMagicConfigs.MotionMagicAcceleration = 
            400; 
        // Target Jerk of 4000 rps/s/s
        shooterMotionMagicConfigs.MotionMagicJerk = 4000; 
        // apply motor configurations 
        forwardMotor.getConfigurator().apply(shooterConfig);
        inverseMotor.getConfigurator().apply(shooterConfig);
        // create Motion Magic Velocity request for shooter 
        shooterMotionMagicVelocityTorqueCurrentFOCRequest =  new MotionMagicVelocityTorqueCurrentFOC(0);
    }

    public void runShooter(double rpm) {
         IS_SHOOTING = true;
        forwardMotor.setControl(
            shooterMotionMagicVelocityTorqueCurrentFOCRequest.withVelocity(rpm)
        );
        inverseMotor.setControl(
            shooterMotionMagicVelocityTorqueCurrentFOCRequest.withVelocity(-rpm)
        );
    }
    
    public void stopShooter() {
         IS_SHOOTING = false; 
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
