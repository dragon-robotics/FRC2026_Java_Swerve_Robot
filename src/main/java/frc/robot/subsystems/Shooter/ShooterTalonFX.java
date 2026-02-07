package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterTalonFX implements ShooterIO {

    private final TalonFX forwardMotor;
    private final TalonFX inverseMotor; 
    
    private final MotionMagicVelocityTorqueCurrentFOC
        shooterMotionMagicVelocityTorqueCurrentFOCRequest;

    public ShooterTalonFX(int canID, TalonFXConfiguration config) {
        forwardMotor = new TalonFX(canID);
        inverseMotor = new TalonFX(canID);        
        // apply motor configurations 
        forwardMotor.getConfigurator().apply(config);
        inverseMotor.getConfigurator().apply(config);
        // create Motion Magic Velocity request for shooter 
        shooterMotionMagicVelocityTorqueCurrentFOCRequest =  new MotionMagicVelocityTorqueCurrentFOC(0);
    }
    
    @Override
    public void runShooter(double rpm) {
         IS_SHOOTING = true;
        forwardMotor.setControl(
            shooterMotionMagicVelocityTorqueCurrentFOCRequest.withVelocity(rpm)
        );
        inverseMotor.setControl(
            shooterMotionMagicVelocityTorqueCurrentFOCRequest.withVelocity(-rpm)
        );
    }
    @Override
    public void stopShooter() {
         IS_SHOOTING = false; 
        runShooter(0.0);
    }

    // returns the average velocity of both motors
    @Override
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
