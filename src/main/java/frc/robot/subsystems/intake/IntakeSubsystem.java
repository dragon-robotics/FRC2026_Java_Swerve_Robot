// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_DEPLOYED_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_DEPLOY_TENSION_CURRENT;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_FAST_PID_SLOT;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_JUICER_FINAL_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_JUICER_PRE_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_POSITION_TOLERANCE;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_SLOW_PID_SLOT;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_STOWED_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_TORQUE_CURRENT;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_TORQUE_CURRENT_MAX_DUTY_CYCLE;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_VOLTAGE;
import static frc.robot.util.constants.IntakeConstants.OUTTAKE_ROLLER_TORQUE_CURRENT;
import static frc.robot.util.constants.IntakeConstants.OUTTAKE_ROLLER_TORQUE_CURRENT_MAX_DUTY_CYCLE;
import static frc.robot.util.constants.IntakeConstants.OUTTAKE_ROLLER_VOLTAGE;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIO.MotorIOInputs;
import frc.robot.io.TorqueCurrentMotorIO;

/**
 * Controls the slapdown intake arm and intake roller.
 *
 * <p>Arm positions are mechanism rotations from the absolute/fused encoder. Roller commands use
 * volts, RPM, or percent output based on the method name. The state machine sends motor commands on
 * state entry, then uses sensor feedback in {@link #periodic()} to advance transition states.
 */
public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeState {
    HOME,
    INTAKE,
    OUTTAKE,
    DEPLOYED,
    DEPLOYING,
    STOWING,
    JUICER
  }

  /** Sub-phases for the JUICER state's timed sequence. */
  public enum JuicerPhase {
    PRE_JUICE,
    SQUEEZE
  }

  protected IntakeState currIntakeState;
  protected IntakeState desiredIntakeState;

  protected final MotorIO intakeRollerLeadIO;
  protected final MotorIO intakeRollerFollowIO;
  protected final MotorIO intakeArmIO;
  protected final MotorIOInputs intakeRollerLeadInputs;
  protected final MotorIOInputs intakeRollerFollowInputs;
  protected final MotorIOInputs intakeArmInputs;

  /** Last state that received entry CAN commands. Reset to null when entering a new state. */
  private IntakeState lastCommandedState = null;

  // Juicer sub-phase tracking; reset to PRE_JUICE on entry.
  private JuicerPhase juicerPhase = JuicerPhase.PRE_JUICE;
  private JuicerPhase lastJuicerPhase = null;

  /**
   * Creates a new intake subsystem.
   *
   * @param intakeRollerLeadIO lead roller motor IO; this motor receives roller commands
   * @param intakeRollerFollowIO follower roller motor IO; updated for telemetry
   * @param intakeArmIO arm motor IO; controls deploy, stow, and juicer positions
   */
  public IntakeSubsystem(
      MotorIO intakeRollerLeadIO, MotorIO intakeRollerFollowIO, MotorIO intakeArmIO) {
    this.intakeRollerLeadIO = intakeRollerLeadIO;
    this.intakeRollerFollowIO = intakeRollerFollowIO;
    this.intakeArmIO = intakeArmIO;
    this.intakeRollerLeadInputs = new MotorIOInputs();
    this.intakeRollerFollowInputs = new MotorIOInputs();
    this.intakeArmInputs = new MotorIOInputs();

    currIntakeState = IntakeState.HOME;
    desiredIntakeState = IntakeState.HOME;
  }

  /* Setters */

  /** Directly commands the lead intake roller velocity in RPM. */
  public void runIntakeRollerRPM(double rpm) {
    intakeRollerLeadIO.setMotorRPM(rpm);
  }

  /** Directly commands the lead intake roller voltage. */
  public void runIntakeRollerVoltage(Voltage voltage) {
    intakeRollerLeadIO.setMotorVoltage(voltage);
  }

  /** Directly commands the lead intake roller percent output from -1.0 to 1.0. */
  public void runIntakeRollerPercentage(double percentage) {
    intakeRollerLeadIO.setMotorPercentage(percentage);
  }

  /** Directly commands the lead intake roller torque current with a duty-cycle cap. */
  public void runIntakeRollerTorqueCurrentFOC(Current torqueCurrent, double maxAbsDutyCycle) {
    double cappedMaxAbsDutyCycle = MathUtil.clamp(maxAbsDutyCycle, 0.0, 1.0);
    if (intakeRollerLeadIO instanceof TorqueCurrentMotorIO torqueCurrentRollerIO) {
      torqueCurrentRollerIO.setMotorTorqueCurrent(torqueCurrent, cappedMaxAbsDutyCycle);
    } else {
      Voltage fallbackVoltage =
          torqueCurrent.in(Amps) > 0 ? INTAKE_ROLLER_VOLTAGE : OUTTAKE_ROLLER_VOLTAGE;
      runIntakeRollerVoltage(fallbackVoltage.times(cappedMaxAbsDutyCycle));
    }
  }

  /** Runs the roller inward with torque-current control. */
  public void runIntakeRollerTorqueCurrentFOC() {
    runIntakeRollerTorqueCurrentFOC(
        INTAKE_ROLLER_TORQUE_CURRENT, INTAKE_ROLLER_TORQUE_CURRENT_MAX_DUTY_CYCLE);
  }

  /** Runs the roller outward with torque-current control. */
  public void runOuttakeRollerTorqueCurrentFOC() {
    runIntakeRollerTorqueCurrentFOC(
        OUTTAKE_ROLLER_TORQUE_CURRENT, OUTTAKE_ROLLER_TORQUE_CURRENT_MAX_DUTY_CYCLE);
  }

  /** Runs the roller inward at the configured intake torque current. */
  public void runIntake() {
    runIntakeRollerTorqueCurrentFOC();
  }

  /** Runs the roller outward at the configured outtake torque current. */
  public void runOuttake() {
    runOuttakeRollerTorqueCurrentFOC();
  }

  /** Stops the intake roller with 0 volts. */
  public void stopIntake() {
    runIntakeRollerVoltage(Volts.of(0.0));
  }

  /**
   * Commands the intake arm to a mechanism position.
   *
   * @param setpoint mechanism rotations from the arm absolute/fused encoder
   * @param slotID closed-loop slot used by the motor controller
   */
  public void setIntakeArmSetpoint(double setpoint, int slotID) {
    intakeArmIO.setMotorPosition(setpoint, slotID);
  }

  /** Commands the arm to the deployed position in mechanism rotations. */
  public void deployIntakeArm() {
    intakeArmIO.setMotorPosition(INTAKE_ARM_DEPLOYED_POSITION, INTAKE_ARM_FAST_PID_SLOT);
  }

  /**
   * Holds the deployed arm down while intaking.
   *
   * <p>If the arm is not deployed yet, this commands the deployed position first. TalonFX-backed
   * arms use torque current in amps after reaching deployed; other motor IO implementations fall
   * back to position hold because not every controller supports torque-current control.
   */
  public void tensionDeployedIntakeArm() {
    if (!isIntakeArmAtDeployed()) {
      deployIntakeArm();
      return;
    }
    if (intakeArmIO instanceof TorqueCurrentMotorIO torqueCurrentArmIO) {
      torqueCurrentArmIO.setMotorTorqueCurrent(INTAKE_ARM_DEPLOY_TENSION_CURRENT);
    } else {
      deployIntakeArm();
    }
  }

  /**
   * Releases the arm motor to neutral output.
   *
   * <p>For this slapdown intake, gravity holds the arm down after deploy. Use only when coasting
   * the arm is intentional.
   */
  public void coastIntakeArm() {
    intakeArmIO.setMotorVoltage(Volts.of(0.0));
  }

  /** Commands the arm to the stowed position in mechanism rotations. */
  public void stowIntakeArm() {
    intakeArmIO.setMotorPosition(INTAKE_ARM_STOWED_POSITION, INTAKE_ARM_SLOW_PID_SLOT);
  }

  /* Getters */

  public IntakeState getCurrentState() {
    return this.currIntakeState;
  }

  public IntakeState getDesiredState() {
    return this.desiredIntakeState;
  }

  public double getIntakeArmSetpoint() {
    return intakeArmInputs.getMotorPosition();
  }

  public double getIntakeRollerSpeed() {
    return intakeRollerLeadInputs.getMotorVelocity();
  }

  /** Returns true when arm position is within configured tolerance of deployed rotations. */
  public boolean isIntakeArmAtDeployed() {
    double positionError =
        Math.abs(INTAKE_ARM_DEPLOYED_POSITION - intakeArmInputs.getMotorPosition());
    return positionError < INTAKE_ARM_POSITION_TOLERANCE;
  }

  /** Returns true when arm position is within configured tolerance of stowed rotations. */
  public boolean isIntakeArmAtStowed() {
    double positionError =
        Math.abs(INTAKE_ARM_STOWED_POSITION - intakeArmInputs.getMotorPosition());
    return positionError < INTAKE_ARM_POSITION_TOLERANCE;
  }

  /** Returns true when arm position is within configured tolerance of pre-juice rotations. */
  public boolean isIntakeArmAtPreJuice() {
    double positionError =
        Math.abs(INTAKE_ARM_JUICER_PRE_POSITION - intakeArmInputs.getMotorPosition());
    return positionError < INTAKE_ARM_POSITION_TOLERANCE;
  }

  public boolean isIntaking() {
    return getIntakeRollerSpeed() > 5;
  }

  public boolean isOuttaking() {
    return getIntakeRollerSpeed() < -5;
  }

  public double getIntakeRollerCurrent() {
    return intakeRollerLeadInputs.getMotorCurrent();
  }

  public double getIntakeArmCurrent() {
    return intakeArmInputs.getMotorCurrent();
  }

  /* State Management */

  private boolean canStartRollerStateImmediately() {
    return currIntakeState == IntakeState.INTAKE
        || currIntakeState == IntakeState.OUTTAKE
        || currIntakeState == IntakeState.DEPLOYED;
  }

  /** Requests an intake state; transition states finish inside {@link #handleStateTransition()}. */
  public void setDesiredState(IntakeState state) {
    this.desiredIntakeState = state;

    if (this.currIntakeState == state) {
      return;
    }

    // Force one-shot CAN commands to resend after state changes.
    lastCommandedState = null;

    switch (state) {
      case HOME -> currIntakeState = IntakeState.STOWING;
      case INTAKE, OUTTAKE -> currIntakeState =
          canStartRollerStateImmediately() ? state : IntakeState.DEPLOYING;
      case DEPLOYED -> currIntakeState = IntakeState.DEPLOYING;
      case JUICER -> {
        currIntakeState = IntakeState.JUICER;
        // Always restart the juicer sequence from PRE_JUICE on (re-)entry
        juicerPhase = JuicerPhase.PRE_JUICE;
        lastJuicerPhase = null;
      }
      default -> {}
    }
  }

  /** Advances the intake state machine and sends hardware commands on state entry. */
  public void handleStateTransition() {
    switch (currIntakeState) {
      case HOME -> handleHomeState();
      case INTAKE -> handleIntakeState();
      case OUTTAKE -> handleOuttakeState();
      case DEPLOYED -> handleDeployedState();
      case DEPLOYING -> handleDeployingState();
      case STOWING -> handleStowingState();
      case JUICER -> handleJuicerState();
    }
  }

  private void markStateEntryHandled() {
    lastCommandedState = currIntakeState;
  }

  private boolean isStateEntry() {
    return lastCommandedState != currIntakeState;
  }

  private void handleHomeState() {
    if (!isStateEntry()) {
      return;
    }

    stowIntakeArm();
    stopIntake();
    markStateEntryHandled();
  }

  private void handleIntakeState() {
    if (!isStateEntry()) {
      return;
    }

    tensionDeployedIntakeArm();
    runIntake();
    markStateEntryHandled();
  }

  private void handleOuttakeState() {
    if (!isStateEntry()) {
      return;
    }

    deployIntakeArm();
    runOuttake();
    markStateEntryHandled();
  }

  private void handleDeployedState() {
    if (!isStateEntry()) {
      return;
    }

    deployIntakeArm();
    stopIntake();
    markStateEntryHandled();
  }

  private void handleDeployingState() {
    if (isStateEntry()) {
      deployIntakeArm();
      runDeployingRoller();
      markStateEntryHandled();
    }

    if (!isIntakeArmAtDeployed()) {
      return;
    }

    lastCommandedState = null;
    currIntakeState = deployedTargetState();
    if (currIntakeState == IntakeState.INTAKE) {
      tensionDeployedIntakeArm();
    }
  }

  private void runDeployingRoller() {
    if (DriverStation.isAutonomous()) {
      runIntakeRollerVoltage(OUTTAKE_ROLLER_VOLTAGE.times(0.5));
    } else {
      stopIntake();
    }
  }

  private IntakeState deployedTargetState() {
    return switch (desiredIntakeState) {
      case INTAKE -> IntakeState.INTAKE;
      case OUTTAKE -> IntakeState.OUTTAKE;
      default -> IntakeState.DEPLOYED;
    };
  }

  private void handleStowingState() {
    if (isStateEntry()) {
      stowIntakeArm();
      stopIntake();
      markStateEntryHandled();
    }

    if (isIntakeArmAtStowed()) {
      lastCommandedState = null;
      currIntakeState = IntakeState.HOME;
    }
  }

  private void handleJuicerState() {
    if (isStateEntry()) {
      runIntakeRollerTorqueCurrentFOC(INTAKE_ROLLER_TORQUE_CURRENT, 0.5);
      lastJuicerPhase = null;
      markStateEntryHandled();
    }

    switch (juicerPhase) {
      case PRE_JUICE -> {
        handlePreJuicePhase();
        if (isIntakeArmAtPreJuice()) {
          juicerPhase = JuicerPhase.SQUEEZE;
        }
      }
      case SQUEEZE -> handleSqueezePhase();
    }
  }

  private boolean isJuicerPhaseEntry() {
    return lastJuicerPhase != juicerPhase;
  }

  private void handlePreJuicePhase() {
    if (!isJuicerPhaseEntry()) {
      return;
    }

    setIntakeArmSetpoint(INTAKE_ARM_JUICER_PRE_POSITION, INTAKE_ARM_FAST_PID_SLOT);
    lastJuicerPhase = juicerPhase;
  }

  private void handleSqueezePhase() {
    if (!isJuicerPhaseEntry()) {
      return;
    }

    setIntakeArmSetpoint(INTAKE_ARM_JUICER_FINAL_POSITION, INTAKE_ARM_SLOW_PID_SLOT);
    lastJuicerPhase = juicerPhase;
  }

  @Override
  public void periodic() {
    handleStateTransition();

    // This method will be called once per scheduler run
    intakeRollerLeadIO.updateInputs(intakeRollerLeadInputs);
    intakeRollerFollowIO.updateInputs(intakeRollerFollowInputs);
    intakeArmIO.updateInputs(intakeArmInputs);

    DogLog.log("Intake/CurrentState", currIntakeState.toString());
  }
}
