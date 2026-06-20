// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_DEPLOYED_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_FAST_PID_SLOT;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_JUICER_FINAL_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_JUICER_PRE_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_POSITION_TOLERANCE;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_SLOW_PID_SLOT;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_STOWED_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_VOLTAGE;
import static frc.robot.util.constants.IntakeConstants.OUTTAKE_ROLLER_VOLTAGE;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    JUICER
  }

  /** Sub-phases for the JUICER state's timed sequence. */
  public enum JuicerPhase {
    PRE_JUICE,
    SQUEEZE
  }

  protected IntakeState currIntakeState;
  protected IntakeState desiredIntakeState;

  protected final MotorIO intakeRollerLeadIO, intakeRollerFollowIO;
  protected final MotorIO intakeArmIO;
  protected final MotorIOInputs intakeRollerLeadInputs, intakeRollerFollowInputs;
  protected final MotorIOInputs intakeArmInputs;

  /**
   * Tracks the last state for which CAN commands were sent, so we can skip redundant writes when
   * the state hasn't changed. Reset to null on any state transition.
   */
  private IntakeState lastCommandedState = null;

  // Juicer sub-phase tracking — always reset to WAIT on (re-)entry
  private JuicerPhase juicerPhase = JuicerPhase.PRE_JUICE;
  private JuicerPhase lastJuicerPhase = null;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(
      MotorIO intakeRollerLeadIO, MotorIO intakeRollerFollowIO, MotorIO intakeArmIO) {
    this.intakeRollerLeadIO = intakeRollerLeadIO;
    this.intakeRollerFollowIO = intakeRollerFollowIO;
    this.intakeArmIO = intakeArmIO;
    this.intakeRollerLeadInputs = new MotorIOInputs();
    this.intakeRollerFollowInputs = new MotorIOInputs();
    this.intakeArmInputs = new MotorIOInputs();

    /* Initialize intake states */
    currIntakeState = IntakeState.HOME;
    desiredIntakeState = IntakeState.HOME;
  }

  /* Setters */

  public void runIntakeRollerRPM(double rpm) {
    intakeRollerLeadIO.setMotorRPM(rpm);
  }

  public void runIntakeRollerVoltage(Voltage voltage) {
    intakeRollerLeadIO.setMotorVoltage(voltage);
  }

  public void runIntakeRollerPercentage(double percentage) {
    intakeRollerLeadIO.setMotorPercentage(percentage);
  }

  public void runIntake() {
    runIntakeRollerVoltage(INTAKE_ROLLER_VOLTAGE);
  }

  public void runOuttake() {
    runIntakeRollerVoltage(OUTTAKE_ROLLER_VOLTAGE);
  }

  public void stopIntake() {
    intakeRollerLeadIO.setMotorVoltage(Volts.of(0.0));
  }

  public void setIntakeArmSetpoint(double setpoint, int slotID) {
    intakeArmIO.setMotorPosition(setpoint, slotID);
  }

  public void deployIntakeArm() {
    intakeArmIO.setMotorPosition(INTAKE_ARM_DEPLOYED_POSITION, INTAKE_ARM_FAST_PID_SLOT);
  }

  /**
   * Release the arm motor to neutral output (0%). For a slapdown intake, gravity holds the arm down
   * once deployed — no PID needed to maintain the down position.
   */
  public void coastIntakeArm() {
    intakeArmIO.setMotorVoltage(Volts.of(0.0));
  }

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

  public boolean isIntakeArmAtDeployed() {
    double positionError =
        Math.abs(INTAKE_ARM_DEPLOYED_POSITION - intakeArmInputs.getMotorPosition());
    return positionError < INTAKE_ARM_POSITION_TOLERANCE;
  }

  public boolean isIntakeArmAtStowed() {
    double positionError =
        Math.abs(INTAKE_ARM_STOWED_POSITION - intakeArmInputs.getMotorPosition());
    return positionError < INTAKE_ARM_POSITION_TOLERANCE;
  }

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
      case HOME -> currIntakeState = IntakeState.STOWING;
      case INTAKE -> {
        if (currIntakeState == IntakeState.OUTTAKE
            || currIntakeState == IntakeState.DEPLOYED
            || currIntakeState == IntakeState.INTAKE) {
          currIntakeState = IntakeState.INTAKE;
        } else {
          currIntakeState = IntakeState.DEPLOYING;
        }
      }
      case OUTTAKE -> {
        if (currIntakeState == IntakeState.INTAKE
            || currIntakeState == IntakeState.DEPLOYED
            || currIntakeState == IntakeState.OUTTAKE) {
          currIntakeState = IntakeState.OUTTAKE;
        } else {
          currIntakeState = IntakeState.DEPLOYING;
        }
      }
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

  public void handleStateTransition() {
    // Handle the state transitions
    switch (currIntakeState) {
      case HOME -> {
        if (lastCommandedState != currIntakeState) {
          // Command arm to stow via PID and stop rollers on state entry.
          stowIntakeArm();
          stopIntake();
          lastCommandedState = currIntakeState;
        }
      }
      case INTAKE -> {
        if (lastCommandedState != currIntakeState) {
          deployIntakeArm();
          runIntake();
          lastCommandedState = currIntakeState;
        }
      }
      case OUTTAKE -> {
        if (lastCommandedState != currIntakeState) {
          deployIntakeArm();
          runOuttake();
          lastCommandedState = currIntakeState;
        }
      }
      case DEPLOYED -> {
        if (lastCommandedState != currIntakeState) {
          deployIntakeArm();
          stopIntake();
          lastCommandedState = currIntakeState;
        }
      }
      case DEPLOYING -> {
        if (lastCommandedState != currIntakeState) {
          deployIntakeArm();
          if (DriverStation.isAutonomous()) {
            runIntakeRollerVoltage(
                OUTTAKE_ROLLER_VOLTAGE.times(
                    0.5)); // Start rollers at half speed when deploying in auto
            // to help get the first ball in quicker
          } else if (DriverStation.isTeleop()) {
            stopIntake();
          } else {
            stopIntake();
          }
          lastCommandedState = currIntakeState;
        }
        // When intake arm reaches setpoint, transition to INTAKE or OUTTAKE state
        // depending on desired state
        if (isIntakeArmAtDeployed()) {
          lastCommandedState = null; // Reset so next state sends commands
          currIntakeState =
              switch (desiredIntakeState) {
                case INTAKE -> IntakeState.INTAKE;
                case OUTTAKE -> IntakeState.OUTTAKE;
                default -> IntakeState.DEPLOYED;
              };
        }
      }
      case STOWING -> {
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
      }
      case JUICER -> {
        if (lastCommandedState != currIntakeState) {
          runIntakeRollerVoltage(
              INTAKE_ROLLER_VOLTAGE.times(0.5)); // Start rollers at half speed when moving to
          lastCommandedState = currIntakeState;
          lastJuicerPhase = null; // Reset so first phase sends its arm command
        }
        switch (juicerPhase) {
          case PRE_JUICE -> {
            if (lastJuicerPhase != juicerPhase) {
              // Move arm quickly to pre-juice setpoint to clear hopper wall
              setIntakeArmSetpoint(INTAKE_ARM_JUICER_PRE_POSITION, INTAKE_ARM_FAST_PID_SLOT);
              // pre-juice position
              // to help get the first ball in quicker
              lastJuicerPhase = juicerPhase;
            }
            if (isIntakeArmAtPreJuice()) {
              juicerPhase = JuicerPhase.SQUEEZE;
            }
          }
          case SQUEEZE -> {
            if (lastJuicerPhase != juicerPhase) {
              // Slowly move arm to final juice position to squeeze remaining balls
              setIntakeArmSetpoint(INTAKE_ARM_JUICER_FINAL_POSITION, INTAKE_ARM_SLOW_PID_SLOT);
              // help get rid of balls
              // from the intake during juicing.
              lastJuicerPhase = juicerPhase;
            }
          }
        }
      }
    }
    // ── Steady states: only send CAN commands on state entry ──
    // ── Transition states: send arm + roller commands once on entry ──
    // TalonFX maintains its onboard PID loop once commanded,
    // so we only need to send the position setpoint once.
    // We still check feedback each loop to know when to transition.
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
