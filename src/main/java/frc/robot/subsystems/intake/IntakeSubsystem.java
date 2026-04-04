// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.util.constants.IntakeConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIO.MotorIOInputs;

public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeState {
    HOME,
    INTAKE,
    OUTTAKE,
    DEPLOYED,
    DEPLOYING,
    STOWING,
    WOKTOSS,
    WOKTOSSING,
    AUTO_WOKTOSSING,
    JUICER
  }

  /** Sub-phases for the JUICER state's timed sequence. */
  public enum JuicerPhase {
    PRE_JUICE,
    SQUEEZE
  }

  protected IntakeState currIntakeState;
  protected IntakeState desiredIntakeState;

  protected final MotorIO intakeRollerIO;
  protected final MotorIO intakeArmIO;
  protected final MotorIOInputs intakeRollerInputs;
  protected final MotorIOInputs intakeArmInputs;

  protected boolean wokTossMovingToDeployed;

  /**
   * Tracks the last state for which CAN commands were sent, so we can skip
   * redundant writes when
   * the state hasn't changed. Reset to null on any state transition.
   */
  private IntakeState lastCommandedState = null;

  /** True once the arm has reached stow in HOME and we've switched to coast. */
  private boolean homeCoasting = false;

  // Juicer sub-phase tracking — always reset to WAIT on (re-)entry
  private JuicerPhase juicerPhase = JuicerPhase.PRE_JUICE;
  private JuicerPhase lastJuicerPhase = null;

  private final NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Intake");

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(MotorIO intakeRollerIO, MotorIO intakeArmIO) {

    this.intakeRollerIO = intakeRollerIO;
    this.intakeArmIO = intakeArmIO;
    this.intakeRollerInputs = new MotorIOInputs();
    this.intakeArmInputs = new MotorIOInputs();

    /* Initialize intake states */
    currIntakeState = IntakeState.HOME;
    desiredIntakeState = IntakeState.HOME;

    /* Initialize wok toss state */
    wokTossMovingToDeployed = false;
  }

  /* Setters */

  public void runIntakeRollerRPM(double rpm) {
    intakeRollerIO.setMotorRPM(rpm);
  }

  public void runIntakeRollerVoltage(double voltage) {
    intakeRollerIO.setMotorVoltage(voltage);
  }

  public void runIntakeRollerPercentage(double percentage) {
    intakeRollerIO.setMotorPercentage(percentage);
  }

  public void runIntake() {
    // runIntakeRollerPercentage(INTAKE_ROLLER_DUTY_CYCLE);
    intakeRollerIO.setMotorPercentage(INTAKE_ROLLER_DUTY_CYCLE);
  }

  public void runOuttake() {
    intakeRollerIO.setMotorPercentage(OUTTAKE_ROLLER_DUTY_CYCLE);
  }

  public void stopIntake() {
    intakeRollerIO.setMotorPercentage(0.0);
  }

  public void setIntakeArmSetpoint(double setpoint, int slotID) {
    intakeArmIO.setMotorPosition(setpoint, slotID);
  }

  public void deployIntakeArm() {
    intakeArmIO.setMotorPosition(INTAKE_ARM_DEPLOYED_POSITION, INTAKE_ARM_FAST_PID_SLOT);
    // intakeRollerIO.setMotorPercentage(0.5);
  }

  /**
   * Release the arm motor to neutral output (0%). For a slapdown intake, gravity
   * holds the arm down
   * once deployed — no PID needed to maintain the down position.
   */
  public void coastIntakeArm() {
    intakeArmIO.setMotorPercentage(0.0);
  }

  public void stowIntakeArm() {
    intakeArmIO.setMotorPosition(INTAKE_ARM_STOWED_POSITION, INTAKE_ARM_SLOW_PID_SLOT);
    // intakeRollerIO.setMotorPercentage(0.5);
  }

  public void wokTossIntakeArm() {
    intakeArmIO.setMotorPosition(INTAKE_ARM_WOKTOSS_POSITION, INTAKE_ARM_SLOW_PID_SLOT);
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
    return intakeRollerInputs.getMotorVelocity();
  }

  public boolean isIntakeArmAtDeployed() {
    double positionError = Math.abs(INTAKE_ARM_DEPLOYED_POSITION - intakeArmInputs.getMotorPosition());
    return positionError < INTAKE_ARM_POSITION_TOLERANCE;
  }

  public boolean isIntakeArmAtStowed() {
    double positionError = Math.abs(INTAKE_ARM_STOWED_POSITION - intakeArmInputs.getMotorPosition());
    return positionError < INTAKE_ARM_POSITION_TOLERANCE;
  }

  public boolean isIntakeArmAtWokToss() {
    double positionError = Math.abs(INTAKE_ARM_WOKTOSS_POSITION - intakeArmInputs.getMotorPosition());
    return positionError < INTAKE_ARM_POSITION_TOLERANCE;
  }

  public boolean isIntakeArmAtPreJuice() {
    double positionError = Math.abs(INTAKE_ARM_JUICER_PRE_POSITION - intakeArmInputs.getMotorPosition());
    return positionError < INTAKE_ARM_POSITION_TOLERANCE;
  }

  public boolean isIntaking() {
    return getIntakeRollerSpeed() > 5;
  }

  public boolean isOuttaking() {
    return getIntakeRollerSpeed() < -5;
  }

  public double getIntakeRollerCurrent() {
    return intakeRollerInputs.getMotorCurrent();
  }

  public double getIntakeArmCurrent() {
    return intakeArmInputs.getMotorCurrent();
  }

  /* State Management */

  public void setDesiredState(IntakeState state) {
    this.desiredIntakeState = state;

    if (this.currIntakeState == state) {
      return;
    }

    // State is changing — reset the guard so the new state sends CAN commands on
    // entry
    lastCommandedState = null;

    // Handle state transitions
    switch (desiredIntakeState) {
      case HOME:
        currIntakeState = IntakeState.STOWING;
        break;
      case INTAKE:
        if (currIntakeState == IntakeState.OUTTAKE
            || currIntakeState == IntakeState.DEPLOYED
            || currIntakeState == IntakeState.INTAKE) {
          currIntakeState = IntakeState.INTAKE;
        } else {
          currIntakeState = IntakeState.DEPLOYING;
        }
        break;
      case OUTTAKE:
        if (currIntakeState == IntakeState.INTAKE
            || currIntakeState == IntakeState.DEPLOYED
            || currIntakeState == IntakeState.OUTTAKE) {
          currIntakeState = IntakeState.OUTTAKE;
        } else {
          currIntakeState = IntakeState.DEPLOYING;
        }
        break;
      case DEPLOYED:
        currIntakeState = IntakeState.DEPLOYING;
        break;
      case WOKTOSS:
        currIntakeState = IntakeState.WOKTOSSING;
        wokTossMovingToDeployed = true;
        break;
      case AUTO_WOKTOSSING:
        currIntakeState = IntakeState.AUTO_WOKTOSSING;
        wokTossMovingToDeployed = true;
        break;
      case JUICER:
        currIntakeState = IntakeState.JUICER;
        // Always restart the juicer sequence from PRE_JUICE on (re-)entry
        juicerPhase = JuicerPhase.PRE_JUICE;
        lastJuicerPhase = null;
        break;
      default:
        break;
    }
  }

  public void handleStateTransition() {
    // Handle the state transitions
    switch (currIntakeState) {
      // ── Steady states: only send CAN commands on state entry ──
      case HOME:
        if (lastCommandedState != currIntakeState) {
          // First entry -- command arm to stow via PID and stop rollers
          stowIntakeArm();
          stopIntake();
          lastCommandedState = currIntakeState;
          homeCoasting = false;
        } else if (!homeCoasting && isIntakeArmAtStowed()) {
          // Arm has arrived at stow -- coast to save 8-14A of stator current.
          // Gravity + mechanical stops hold the arm in place; PID is unnecessary.
          coastIntakeArm();
          homeCoasting = true;
        }
        break;
      case INTAKE:
        if (lastCommandedState != currIntakeState) {
          coastIntakeArm();
          runIntake();
          lastCommandedState = currIntakeState;
        }
        break;
      case OUTTAKE:
        if (lastCommandedState != currIntakeState) {
          coastIntakeArm();
          runOuttake();
          lastCommandedState = currIntakeState;
        }
        break;
      case DEPLOYED:
        if (lastCommandedState != currIntakeState) {
          coastIntakeArm();
          stopIntake();
          lastCommandedState = currIntakeState;
        }
        break;
      case WOKTOSS:
        if (lastCommandedState != currIntakeState) {
          wokTossIntakeArm();
          runIntake();
          lastCommandedState = currIntakeState;
        }
        break;

      // ── Transition states: send arm + roller commands once on entry ──
      // TalonFX maintains its onboard PID loop once commanded,
      // so we only need to send the position setpoint once.
      // We still check feedback each loop to know when to transition.
      case DEPLOYING:
        if (lastCommandedState != currIntakeState) {
          deployIntakeArm();
          stopIntake();
          lastCommandedState = currIntakeState;
        }
        // When intake arm reaches setpoint, transition to INTAKE or OUTTAKE state
        // depending on desired state
        if (isIntakeArmAtDeployed()) {
          lastCommandedState = null; // Reset so next state sends commands
          if (desiredIntakeState == IntakeState.INTAKE) {
            currIntakeState = IntakeState.INTAKE;
          } else if (desiredIntakeState == IntakeState.OUTTAKE) {
            currIntakeState = IntakeState.OUTTAKE;
          } else {
            currIntakeState = IntakeState.DEPLOYED;
          }
        }
        break;
      case STOWING:
        if (lastCommandedState != currIntakeState) {
          stowIntakeArm();
          stopIntake();
          lastCommandedState = currIntakeState;
        }
        // When intake arm reaches setpoint, transition to HOME state
        if (isIntakeArmAtStowed()) {
          lastCommandedState = null; // Reset so next state sends commands
          currIntakeState = IntakeState.HOME;
        }
        break;
      case WOKTOSSING:
        if (lastCommandedState != currIntakeState) {
          wokTossIntakeArm();
          runIntake();
          lastCommandedState = currIntakeState;
        }
        // When intake arm reaches setpoint, transition to WOKTOSS state
        if (isIntakeArmAtWokToss()) {
          lastCommandedState = null; // Reset so next state sends commands
          currIntakeState = IntakeState.WOKTOSS;
        }
        break;
      case AUTO_WOKTOSSING:
        // Intake arm oscillate between deployed and woktoss setpoints in a set sequence
        // 1. Do nothing for the first 3 seconds
        // 2. Then oscillate between deployed and woktoss setpoints every 0.5 seconds
        break;
      case JUICER:
        if (lastCommandedState != currIntakeState) {
          lastCommandedState = currIntakeState;
          lastJuicerPhase = null; // Reset so first phase sends its arm command
        }
        switch (juicerPhase) {
          case PRE_JUICE:
            if (lastJuicerPhase != juicerPhase) {
              // Move arm quickly to pre-juice setpoint to clear hopper wall
              setIntakeArmSetpoint(INTAKE_ARM_JUICER_PRE_POSITION, INTAKE_ARM_FAST_PID_SLOT);
              lastJuicerPhase = juicerPhase;
            }
            if (isIntakeArmAtPreJuice()) {
              juicerPhase = JuicerPhase.SQUEEZE;
            }
            break;
          case SQUEEZE:
            if (lastJuicerPhase != juicerPhase) {
              // Slowly move arm to final juice position to squeeze remaining balls
              setIntakeArmSetpoint(INTAKE_ARM_JUICER_FINAL_POSITION, INTAKE_ARM_SLOW_PID_SLOT);
              runIntakeRollerPercentage(
                  0.5); // Start rollers at 50% when we start squeezing to help get rid of balls
              // from the intake during juicing.
              lastJuicerPhase = juicerPhase;
            }
            break;
        }
        break;
    }
  }

  @Override
  public void periodic() {
    DogLog.time("Perf/Intake");
    handleStateTransition();

    // This method will be called once per scheduler run
    // intakeRollerIO.updateInputs(intakeRollerInputs);
    intakeArmIO.updateInputs(intakeArmInputs);

    DogLog.log("Intake/CurrentState", currIntakeState.toString());
    DogLog.timeEnd("Perf/Intake");
  }
}
