package frc.robot.subsystems.hopper;

import static frc.robot.util.constants.HopperConstants.*;

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

  private final HopperIO leadRollerMotorIO;
  private final HopperIO followRollerMotorIO;

  // hardware layer
  private final HopperIOInputs leadRollerMotorIOInputs;
  private final HopperIOInputs followRollerMotorIOInputs;

  /* Creates new HopperSubsystem */
  public HopperSubsystem(HopperIO leadRollerMotorIO, HopperIO followRollerMotorIO) {

    this.leadRollerMotorIO = leadRollerMotorIO;
    this.followRollerMotorIO = followRollerMotorIO;

    this.leadRollerMotorIOInputs = new HopperIOInputs();
    this.followRollerMotorIOInputs = new HopperIOInputs();

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
        currHopperState = HopperState.INDEXTOINTAKE;
        break;
      case INDEXTOSHOOTER:
        currHopperState = HopperState.INDEXTOSHOOTER;
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
    // leadRollerMotorIO.setMotorRPM(HOPPER_ROLLER_RPM);
    leadRollerMotorIO.setMotorPercentage(HOPPER_ROLLER_DUTY_CYCLE);
  }

  public void indexToIntake() {
    // leadRollerMotorIO.setMotorRPM(-HOPPER_ROLLER_RPM);
    leadRollerMotorIO.setMotorPercentage(-HOPPER_ROLLER_DUTY_CYCLE);
  }

  public void stopHopperRoller() {
    // leadRollerMotorIO.setMotorRPM(0);
    leadRollerMotorIO.setMotorPercentage(0);
  }

  public void handleStateTransition() {

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

    /* This method will be called once per scheduler run */
    handleStateTransition();

    DogLog.log("Hopper/Hopper State", currHopperState.toString());

    leadRollerMotorIO.updateInputs(leadRollerMotorIOInputs);
    followRollerMotorIO.updateInputs(followRollerMotorIOInputs);
  }
}
