package frc.robot.subsystems.climber;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class ClimberIOTalonFX implements ClimberIO {

    private final TalonFX leadMotor;
    private final TalonFX followerMotor;
    private final MotionMagicExpoTorqueCurrentFOC 
        climberMotionMagicTorqueCurrentFOC;  

    public ClimberIOTalonFX(int canID, TalonFXConfiguration config) {
        leadMotor = new TalonFX(canID);
        followerMotor = new TalonFX(canID);
        // Apply configurations to lead motor
        leadMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);

        // Set follower to follow lead motor
        followerMotor.setControl(new Follower(leadMotor.getDeviceID(), MotorAlignmentValue.Aligned));   

        // Create control requests
        climberMotionMagicTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0.0);
    }

    @Override 
    public void setElevatorSetpoint(double setPoint) {
        leadMotor.setControl(
            climberMotionMagicTorqueCurrentFOC.withPosition(setPoint)
        );
    }
    
    /* getters */
    @Override
    public double getElevatorPosition() {
        return leadMotor.getPosition().getValueAsDouble();
    }

    @Override
    public boolean isElevatorStowed(double homeSetPoint, double tolerance) {
        return Math.abs(getElevatorPosition() - homeSetPoint) < tolerance;
    }

    @Override
    public void updateInputs(ClimberIOInputs climberInputs) {
        // Motor connection status
        climberInputs.setLeadMotorConnected(leadMotor.isConnected());
        climberInputs.setFollowerMotorConnected(followerMotor.isConnected());

        // Lead motor data
        climberInputs.setLeadMotorPosition(leadMotor.getPosition().getValueAsDouble());
        climberInputs.setLeadMotorVelocity(leadMotor.getVelocity().getValueAsDouble());
        climberInputs.setLeadMotorVoltage(leadMotor.getMotorVoltage().getValueAsDouble());
        climberInputs.setLeadMotorCurrent(leadMotor.getStatorCurrent().getValueAsDouble());
        climberInputs.setLeadMotorTemperature(leadMotor.getDeviceTemp().getValueAsDouble());

        // Follower motor data
        climberInputs.setFollowerMotorPosition(followerMotor.getPosition().getValueAsDouble());
        climberInputs.setFollowerMotorVelocity(followerMotor.getVelocity().getValueAsDouble());
        climberInputs.setFollowerMotorVoltage(followerMotor.getMotorVoltage().getValueAsDouble());
        climberInputs.setFollowerMotorCurrent(followerMotor.getStatorCurrent().getValueAsDouble());
        climberInputs.setFollowerMotorTemperature(followerMotor.getDeviceTemp().getValueAsDouble());
    }
}
