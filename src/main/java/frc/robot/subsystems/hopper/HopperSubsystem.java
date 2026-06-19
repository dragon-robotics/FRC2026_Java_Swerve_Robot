package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.constants.HopperConstants.HOPPER_ROLLER_MAX_VOLTAGE;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIO.MotorIOInputs;

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

  public void runHopperRollerVoltage(Voltage voltage) {
    leadRollerMotorIO.setMotorVoltage(voltage);
  }

  public void runHopperRollerPercentage(double percentage) {
    leadRollerMotorIO.setMotorPercentage(percentage);
  }

  public void indexToShooter() {
    leadRollerMotorIO.setMotorVoltage(HOPPER_ROLLER_MAX_VOLTAGE);
  }

  public void indexToIntake() {
    leadRollerMotorIO.setMotorVoltage(HOPPER_ROLLER_MAX_VOLTAGE.unaryMinus());
  }

  public void stopHopperRoller() {
    leadRollerMotorIO.setMotorVoltage(Volts.of(0.0));
  }

  /* State Management */
  public void setDesiredState(HopperState state) {
    this.currHopperState = state;
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
      case STOP -> /* Stop rollers */ stopHopperRoller();
      case INDEXTOSHOOTER -> /* set hopper rollers to index to shooter speed */ indexToShooter();
      case INDEXTOINTAKE -> /* set hopper rollers to index to outtake speed */ indexToIntake();
      default -> {}
    }
  }

  @Override
  public void periodic() {
    /* This method will be called once per scheduler run */
    handleStateTransition();

    leadRollerMotorIO.updateInputs(leadRollerMotorIOInputs);
    followRollerMotorIO.updateInputs(followRollerMotorIOInputs);

    DogLog.log("Hopper/CurrentState", currHopperState.toString());
  }
}
