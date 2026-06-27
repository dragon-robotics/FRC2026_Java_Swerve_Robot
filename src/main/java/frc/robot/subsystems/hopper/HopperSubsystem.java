package frc.robot.subsystems.hopper;

import static frc.robot.util.constants.HopperConstants.HOPPER_INDEX_TO_INTAKE_VOLTAGE;
import static frc.robot.util.constants.HopperConstants.HOPPER_INDEX_TO_SHOOTER_VOLTAGE;
import static frc.robot.util.constants.HopperConstants.HOPPER_STOP_VOLTAGE;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIO.MotorIOInputs;

/**
 * Controls the hopper roller pair that moves fuel between the intake and shooter.
 *
 * <p>Positive voltage indexes fuel toward the shooter. Negative voltage indexes fuel back toward
 * the intake. State commands are only sent on state entry to avoid redundant CAN writes.
 */
public class HopperSubsystem extends SubsystemBase {

  /** Hopper roller states requested by the superstructure. */
  public enum HopperState {
    STOP,
    INDEXTOSHOOTER,
    INDEXTOINTAKE
  }

  private HopperState currHopperState;
  private HopperState desiredHopperState;

  private final MotorIO leadRollerMotorIO;
  private final MotorIO followRollerMotorIO;

  private final MotorIOInputs leadRollerMotorIOInputs;
  private final MotorIOInputs followRollerMotorIOInputs;

  /** Last state that received entry CAN commands. */
  private HopperState lastCommandedState = null;

  /**
   * Creates a new hopper subsystem.
   *
   * @param leadRollerMotorIO lead roller motor IO; receives hopper commands
   * @param followRollerMotorIO follower roller motor IO; updated for telemetry
   */
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

  /** Returns the current hopper roller state. */
  public HopperState getCurrentState() {
    return currHopperState;
  }

  /** Returns the stored desired hopper state. */
  public HopperState getDesiredState() {
    return desiredHopperState;
  }

  /* Functions */

  /** Directly commands the lead hopper roller velocity in RPM. */
  public void runHopperRollerRPM(double rpm) {
    leadRollerMotorIO.setMotorRPM(rpm);
  }

  /** Directly commands the lead hopper roller voltage. */
  public void runHopperRollerVoltage(Voltage voltage) {
    leadRollerMotorIO.setMotorVoltage(voltage);
  }

  /** Directly commands the lead hopper roller percent output from -1.0 to 1.0. */
  public void runHopperRollerPercentage(double percentage) {
    leadRollerMotorIO.setMotorPercentage(percentage);
  }

  /** Runs the hopper toward the shooter at the configured voltage. */
  public void indexToShooter() {
    runHopperRollerVoltage(HOPPER_INDEX_TO_SHOOTER_VOLTAGE);
  }

  /** Runs the hopper toward the intake at the configured reverse voltage. */
  public void indexToIntake() {
    runHopperRollerVoltage(HOPPER_INDEX_TO_INTAKE_VOLTAGE);
  }

  /** Stops the hopper roller with 0 volts. */
  public void stopHopperRoller() {
    runHopperRollerVoltage(HOPPER_STOP_VOLTAGE);
  }

  /* State Management */

  /** Sets the hopper state executed by {@link #handleStateTransition()}. */
  public void setDesiredState(HopperState state) {
    this.currHopperState = state;
  }

  /** Sends hopper motor commands for the current state on state entry. */
  public void handleStateTransition() {
    if (!isStateEntry()) {
      return;
    }
    markStateEntryHandled();

    switch (currHopperState) {
      case STOP -> stopHopperRoller();
      case INDEXTOSHOOTER -> indexToShooter();
      case INDEXTOINTAKE -> indexToIntake();
      default -> {}
    }
  }

  private boolean isStateEntry() {
    return currHopperState != lastCommandedState;
  }

  private void markStateEntryHandled() {
    lastCommandedState = currHopperState;
  }

  @Override
  public void periodic() {
    handleStateTransition();

    leadRollerMotorIO.updateInputs(leadRollerMotorIOInputs);
    followRollerMotorIO.updateInputs(followRollerMotorIOInputs);

    DogLog.forceNt.log("Hopper/CurrentState", currHopperState.toString());
  }
}
