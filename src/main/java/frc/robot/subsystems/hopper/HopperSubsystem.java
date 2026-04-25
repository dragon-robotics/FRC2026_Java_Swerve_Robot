package frc.robot.subsystems.hopper;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIO.MotorIOInputs;
import static frc.robot.util.constants.HopperConstants.HOPPER_ROLLER_DUTY_CYCLE;

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

  private final MotorIO leadRollerMotorIO;
  private final MotorIO followRollerMotorIO;

  // hardware layer
  private final MotorIOInputs leadRollerMotorIOInputs;
  private final MotorIOInputs followRollerMotorIOInputs;

  /* Creates new HopperSubsystem */
  public HopperSubsystem(MotorIO leadRollerMotorIO, MotorIO followRollerMotorIO) {

    this.leadRollerMotorIO = leadRollerMotorIO;
    this.followRollerMotorIO = followRollerMotorIO;

    this.leadRollerMotorIOInputs = new MotorIOInputs();
    this.followRollerMotorIOInputs = new MotorIOInputs();

    this.currHopperState = HopperState.STOP;
    this.desiredHopperState = HopperState.STOP;
  }

  /* Setters */

  /* Getters */

  public HopperState getCurrentState() {
    return currHopperState;
  }

  public HopperState getDesiredState() {
    return desiredHopperState;
  }

  /* Functions */

  public void runHopperRollerRPM(double rpm) {
    leadRollerMotorIO.setMotorRPM(rpm);
  }

  public void runHopperRollerVoltage(double voltage) {
    leadRollerMotorIO.setMotorVoltage(voltage);
  }

  public void runHopperRollerPercentage(double percentage) {
    leadRollerMotorIO.setMotorPercentage(percentage);
  }

  public void indexToShooter() {
    leadRollerMotorIO.setMotorPercentage(HOPPER_ROLLER_DUTY_CYCLE);
  }

  public void indexToIntake() {
    leadRollerMotorIO.setMotorPercentage(-HOPPER_ROLLER_DUTY_CYCLE);
  }

  public void stopHopperRoller() {
    leadRollerMotorIO.setMotorPercentage(0);
  }

  /* State Managemeent */
  public void setDesiredState(HopperState state) {
    this.currHopperState = state;
    // switch (desiredHopperState) {
    // case STOP:
    // currHopperState = HopperState.STOP;
    // break;
    // case INDEXTOINTAKE:
    // currHopperState = HopperState.INDEXTOINTAKE;
    // break;
    // case INDEXTOSHOOTER:
    // currHopperState = HopperState.INDEXTOSHOOTER;
    // break;
    // default:
    // break;
    // }
  }

  // Track the last state we sent CAN commands for to avoid redundant writes
  private HopperState lastCommandedState = null;

  public void handleStateTransition() {
    // Skip redundant CAN writes if state hasn't changed
    if (currHopperState == lastCommandedState) {
      return;
    }
    lastCommandedState = currHopperState;

    /* Handle the state transitions */
    switch (currHopperState) {
      case STOP:
        /* Stop rollers */
        stopHopperRoller();
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
    DogLog.time("Perf/Hopper");
    /* This method will be called once per scheduler run */
    handleStateTransition();

    DogLog.log("Hopper/Hopper_CurrentState", currHopperState.toString());

    // leadRollerMotorIO.updateInputs(leadRollerMotorIOInputs);
    // followRollerMotorIO.updateInputs(followRollerMotorIOInputs);
    DogLog.timeEnd("Perf/Hopper");
  }
}
