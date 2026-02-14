package frc.robot.subsystems.hopper;

import static frc.robot.Constants.HopperConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperIO.HopperIOInputs;

public class HopperSubsystem extends SubsystemBase {

  public enum HopperState {
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

  public HopperSubsystem(HopperIO rollerMotorIO) {

    this.expandingMotorIO = expandingMotorIO;
    this.rollerMotorIO = rollerMotorIO;
    this.expandingMotorInputs = new HopperIOInputs();
    this.rollerMotorIOInputs = new HopperIOInputs();

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
    return Math.abs(expandingMotorInputs.getExpandingMotorPosition() - targetPosition) < tolerance;
  }

  public void expandHopper() {
    expandingMotorIO.expandHopper(EXPANDING_SETPOINT);
  }

  public void stowHopper() {
    expandingMotorIO.stowHopper();
  }

  /** Index fuel toward shooter */
  public void indexToShooter() {
    rollerMotorIO.expandHopper(ROLLER_RPM);
  }

  /** Index fuel toward intake/outtake */
  public void indexToIntake() {
    rollerMotorIO.expandHopper(-ROLLER_RPM);
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
    rollerMotorIO.updateInputs(rollerMotorIOInputs);
    expandingMotorIO.updateInputs(expandingMotorInputs);

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
    expandingMotorIO.updateInputs(expandingMotorInputs);
    rollerMotorIO.updateInputs(rollerMotorIOInputs);
  }
}
