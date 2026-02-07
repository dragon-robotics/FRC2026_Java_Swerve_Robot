package frc.robot.subsystems.climber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import static frc.robot.Constants.ClimberConstants.*;

public class ClimberSubsystem extends SubsystemBase{
    
    public enum ClimberState {
        STOWED, 
        L1,
        TRANSISTION_L1,
        STOWING,
    }

    private ClimberState currentClimberState;
    private ClimberState desiredClimberState;

    private ClimberIO climberIO;
    private ClimberIOInputs climberInputs; 

    public ClimberSubsystem(ClimberIO climberIO) {
        this.climberIO = climberIO;
        this.climberInputs = new ClimberIOInputs();

        // initailize climber states
        this.desiredClimberState = ClimberState.STOWED;
        this.currentClimberState = ClimberState.STOWED;
    }

    public void setDesiredState(ClimberState state) {
        this.desiredClimberState = state;
        
        switch (desiredClimberState) {
            case STOWED: 
                currentClimberState = ClimberState.STOWING;
                break;
            case L1: 
                currentClimberState = ClimberState.TRANSISTION_L1;
                break;
            default:
                break;
        } 
    }

    public void setElevatorSetpoint(double setPoint) {
        climberIO.setElevatorSetpoint(setPoint);
  }
    /* getters */
    public ClimberState getCurrentState() {
        return currentClimberState;
    }

    public ClimberState getDesiredState() {
        return desiredClimberState;
    }
    public double getElevatorPosition() {
        return climberIO.getElevatorPosition();
    }

    private boolean isAtPosition(double setPoint, double tolerance) {
        return Math.abs(getElevatorPosition() - setPoint) < tolerance;
    }

    public boolean hasClimbed(double setPoint, double tolerance) {
        return Math.abs(setPoint - getElevatorPosition()) < tolerance;
    }
    public boolean isHome(double homeSetPoint, double tolerance) {
        return climberIO.isElevatorStowed(homeSetPoint, tolerance);
    }
    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);
        switch (currentClimberState) {
            case STOWED: 
                break;
            case STOWING:
                setElevatorSetpoint(CLIMBER_RETRACTED_SETPOINT);
                if (isAtPosition(CLIMBER_RETRACTED_SETPOINT, POSITION_TOLERANCE)) {
                    currentClimberState = ClimberState.STOWED;
                }
                break;
            case TRANSISTION_L1:
               setElevatorSetpoint(CLIMBER_L1_SETPOINT);
                if (isAtPosition(CLIMBER_HOME_SETPOINT, POSITION_TOLERANCE)) {
                    currentClimberState = ClimberState.L1;
                }
                break;
            case L1:
                if (desiredClimberState == ClimberState.STOWED) {
                    currentClimberState = ClimberState.STOWING;
                }
                break;
        }
    }
}

