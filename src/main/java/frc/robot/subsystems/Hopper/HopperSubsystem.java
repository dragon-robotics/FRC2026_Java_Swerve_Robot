package frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hopper.HopperIO.HopperIOInputs;
import static frc.robot.Constants.HopperConstants.*; 

public class HopperSubsystem extends SubsystemBase {
    
    public enum HopperState {
        STOWED, 
        INDEXTOSHOOTER, 
        INDEXTOINTAKE, 
        STOWING, 
        DEPLOYING 
    }
    
    private HopperState currHopperState; 
    private HopperState desiredHopperState; 

    private final HopperIO hopperIO; 
    private final HopperIOInputs hopperInputs; 

    public HopperSubsystem(HopperIO hopperIO) {
        this.hopperIO = hopperIO; 
        this.hopperInputs = new HopperIOInputs(); 

        this.currHopperState = HopperState.STOWED;
        this.desiredHopperState = HopperState.STOWED;
    }
    
    public void setDesiredState(HopperState state) {
        this.desiredHopperState = state; 

        switch (desiredHopperState) {
            case STOWED:
                currHopperState = HopperState.STOWING; 
                break;
            case INDEXTOINTAKE:
                if (currHopperState == HopperState.INDEXTOSHOOTER) {
                    currHopperState = HopperState.INDEXTOINTAKE;
                } else {
                    // retracts hopper as we outtake fuel
                    currHopperState = HopperState.STOWING; 
                }
                break;
            case INDEXTOSHOOTER: 
                if (currHopperState == HopperState.INDEXTOINTAKE) {
                    currHopperState = HopperState.INDEXTOSHOOTER; 
                } else {
                    currHopperState = HopperState.DEPLOYING;
                }
                break;
            default:
                break;
        }
    }

    /** Check if expanding motor has reached target position */
    private boolean isDeployed(double targetPosition, double tolerance) {
        return Math.abs(hopperInputs.getExpandingMotorPosition() - targetPosition) < tolerance;
    }

    public void expandHopper() {
        hopperIO.expandHopper(EXPANDING_SETPOINT, ROLLER_RPM);
    }

    public void stowHopper() {
        hopperIO.stowHopper();
    }

    /** Index fuel toward shooter */
    public void indexToShooter() {
        hopperIO.expandHopper(EXPANDING_SETPOINT, ROLLER_RPM);
    }

    /** Index fuel toward intake/outtake */
    public void indexToIntake() {
        hopperIO.expandHopper(0.0, -ROLLER_RPM);
    }

    /* getters */

    public HopperState getCurrentState() {
        return currHopperState;
    }

    public HopperState getDesiredState() {
        return desiredHopperState;
    }

    @Override
    public void periodic() {
        // Update sensor inputs
        hopperIO.updateInputs(hopperInputs);

        // State machine logic
        switch (currHopperState) {
            case STOWED:
                break;
                
            case STOWING:
                stowHopper();
                if (isDeployed(0.0, 0.0)) {
                    currHopperState = HopperState.STOWED;
                }
                break;
                
            case DEPLOYING:
                expandHopper();
                if (isDeployed(EXPANDING_SETPOINT, EXPANDING_TOLERANCE)) {
                    if (desiredHopperState == HopperState.INDEXTOSHOOTER) {
                        currHopperState = HopperState.INDEXTOSHOOTER;
                    } else if (desiredHopperState == HopperState.INDEXTOINTAKE) {
                        currHopperState = HopperState.INDEXTOINTAKE;
                    }
                }
                break;
                
            case INDEXTOSHOOTER:
                indexToShooter();
                break;
                
            case INDEXTOINTAKE:
                indexToIntake();
                break;
                
            default:
                break;
        }
        
        hopperIO.updateInputs(hopperInputs);
    }
}