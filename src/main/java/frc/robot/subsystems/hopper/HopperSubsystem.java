package frc.robot.subsystems.hopper;

import static frc.robot.Constants.HopperConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperIO.HopperIOInputs;

public class HopperSubsystem extends SubsystemBase {

  public enum HopperState {
    STOP,
    INDEXTOSHOOTER,
    INDEXTOINTAKE
  }

  // current state
  private HopperState currHopperState;
  // desired state
  private HopperState desiredHopperState;
  // inputs

  private final HopperIO rollerMotorIO;
  // hardware layer
  private final HopperIOInputs rollerMotorIOInputs;

  /* Creates new HopperSubsystem */
  public HopperSubsystem(HopperIO rollerMotorIO) {

    this.rollerMotorIO = rollerMotorIO;
    this.rollerMotorIOInputs = new HopperIOInputs();

    this.currHopperState = HopperState.STOP;
    this.desiredHopperState = HopperState.STOP;
  }

  /* Setters */

  public void setDesiredState(HopperState state) {
    this.desiredHopperState = state;

    switch (desiredHopperState) {
      case STOP:
        currHopperState = HopperState.STOP;
        break;
      case INDEXTOINTAKE:
        if (currHopperState == HopperState.INDEXTOSHOOTER) {
          currHopperState = HopperState.INDEXTOINTAKE;
        }
        break;
      case INDEXTOSHOOTER:
        if (currHopperState == HopperState.INDEXTOINTAKE) {
          currHopperState = HopperState.INDEXTOSHOOTER;
        }
        break;
      default:
        break;
    }
  }

  /* Getters */

  public HopperState getCurrentState() {
    return currHopperState;
  }

  public HopperState getDesiredState() {
    return desiredHopperState;
  }

  /* Functions */

  public void indexToShooter() {
    rollerMotorIO.setMotorRPM(ROLLER_RPM);
  }

  public void indexToIntake() {
    rollerMotorIO.setMotorRPM(-ROLLER_RPM);
  }

  public void stopIntake() {
    rollerMotorIO.setMotorRPM(0);
  }

  public void handleStateTransition() {
    
    /* Handle the state transitions */
    switch (currHopperState) {
      case STOP:
        /* Stop rollers */
        stopIntake();
        break;
      case INDEXTOSHOOTER:
        /* set hopper rollers to index to shooter speed */
        indexToShooter();
        break;
      case INDEXTOINTAKE:
        /* set hopper rollers to index to outtake speed */
        indexToIntake();
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    
    /* This method will be called once per scheduler run */
    handleStateTransition();

    DogLog.log("Hopper/Hopper State", currHopperState.toString());
    
    rollerMotorIO.updateInputs(rollerMotorIOInputs);
  }
}
