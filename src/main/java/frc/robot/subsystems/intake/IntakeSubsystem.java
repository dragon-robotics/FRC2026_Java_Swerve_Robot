// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.util.constants.IntakeConstants.*;

import dev.doglog.DogLog;
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

  // Juicer sub-phase tracking — always reset to WAIT on (re-)entry
  private JuicerPhase juicerPhase = JuicerPhase.PRE_JUICE;

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
    // DogLog.log("Intake/Roller_Debug", percentage);
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
    double positionError =
        Math.abs(INTAKE_ARM_DEPLOYED_POSITION - intakeArmInputs.getMotorPosition());
    return positionError < INTAKE_ARM_POSITION_TOLERANCE;
  }

  public boolean isIntakeArmAtStowed() {
    double positionError =
        Math.abs(INTAKE_ARM_STOWED_POSITION - intakeArmInputs.getMotorPosition());
    return positionError < INTAKE_ARM_POSITION_TOLERANCE;
  }

  public boolean isIntakeArmAtWokToss() {
    double positionError =
        Math.abs(INTAKE_ARM_WOKTOSS_POSITION - intakeArmInputs.getMotorPosition());
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
        // Always restart the juicer sequence from WAIT on (re-)entry
        juicerPhase = JuicerPhase.PRE_JUICE;
        break;
      default:
        break;
    }
  }

  public void handleStateTransition() {
    // Handle the state transitions
    switch (currIntakeState) {
      case HOME:
        // Set intake arm to home setpoint
        stowIntakeArm();
        // Set intake rollers off
        stopIntake();
        break;
      case INTAKE:
        // Set intake arm to intake setpoint
        deployIntakeArm();
        // // Set intake rollers to intake speed
        // DogLog.log("Intake/Roller Debug", "STATE_TRANSITION");
        runIntake();
        break;
      case OUTTAKE:
        // Set intake arm to intake setpoint
        deployIntakeArm();
        // Set intake rollers to outtake speed
        runOuttake();
        break;
      case DEPLOYED:
        // Set intake arm to intake setpoint
        deployIntakeArm();
        // Set intake rollers off
        stopIntake();
        break;
      case WOKTOSS:
        // Set intake arm to home setpoint
        wokTossIntakeArm();
        // Set intake rollers off
        runIntake();
        break;
      case DEPLOYING:
        // Set intake arm to intake setpoint
        deployIntakeArm();
        // Set intake rollers off
        stopIntake();
        // When intake arm reaches setpoint, transition to INTAKE or OUTTAKE state
        // depending on
        // desired state
        if (isIntakeArmAtDeployed()) {
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
        // Set intake arm to home setpoint
        stowIntakeArm();
        // Set intake rollers off
        stopIntake();
        // When intake arm reaches setpoint, transition to HOME state
        if (isIntakeArmAtStowed()) {
          currIntakeState = IntakeState.HOME;
        }
        break;
      case WOKTOSSING:
        // Set intake arm to home setpoint
        wokTossIntakeArm();
        // Set intake rollers off
        runIntake();
        // When intake arm reaches setpoint, transition to HOME state
        if (isIntakeArmAtWokToss()) {
          currIntakeState = IntakeState.WOKTOSS;
        }
        break;
      case AUTO_WOKTOSSING:
        // Intake arm oscillate between deployed and woktoss setpoints in a set sequence
        // 1. Do nothing for the first 3 seconds
        // 2. Then oscillate between deployed and woktoss setpoints every 0.5 seconds

        // // Set intake arm to home setpoint
        // wokTossIntakeArm();
        // // Set intake rollers off
        // runIntake();
        // // When intake arm reaches setpoint, transition to HOME state
        // if (isIntakeArmAtWokToss()) {
        //   currIntakeState = IntakeState.WOKTOSS;
        // }
        break;
      case JUICER:
        runIntake();
        switch (juicerPhase) {
          case PRE_JUICE:
            // Phase 2: Move arm quickly to pre-juice setpoint to clear hopper wall
            setIntakeArmSetpoint(INTAKE_ARM_JUICER_PRE_POSITION, INTAKE_ARM_FAST_PID_SLOT);
            if (isIntakeArmAtPreJuice()) {
              juicerPhase = JuicerPhase.SQUEEZE;
            }
            break;
          case SQUEEZE:
            // Phase 3: Slowly move arm to final juice position to squeeze remaining balls
            setIntakeArmSetpoint(INTAKE_ARM_JUICER_FINAL_POSITION, INTAKE_ARM_SLOW_PID_SLOT);
            break;
        }
        break;
    }
  }

  @Override
  public void periodic() {
    DogLog.time("Perf/Intake");

    handleStateTransition();

    DogLog.log("Intake/Intake State", currIntakeState.toString());
    DogLog.log("Intake/Juicer Phase", juicerPhase.toString());

    // This method will be called once per scheduler run
    intakeRollerIO.updateInputs(intakeRollerInputs);
    intakeArmIO.updateInputs(intakeArmInputs);

    DogLog.timeEnd("Perf/Intake");
  }
}
