// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeSubsystemConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeState {
    HOME,
    INTAKE,
    OUTTAKE,
    DEPLOYED,
    DEPLOYING,
    STOWING
  }

  protected IntakeState currIntakeState;
  protected IntakeState desiredIntakeState;

  protected final IntakeIO intakeRollerIO;
  protected final IntakeIO intakeArmIO;
  protected final IntakeIOInputs intakeRollerInputs;
  protected final IntakeIOInputs intakeArmInputs;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(IntakeIO intakeRollerIO, IntakeIO intakeArmIO) {

    this.intakeRollerIO = intakeRollerIO;
    this.intakeArmIO = intakeArmIO;
    this.intakeRollerInputs = new IntakeIOInputs();
    this.intakeArmInputs = new IntakeIOInputs();

    /* Initialize intake states */
    currIntakeState = IntakeState.HOME;
    desiredIntakeState = IntakeState.HOME;
  }

  /* Setters */

  public void setDesiredState(IntakeState state) {
    this.desiredIntakeState = state;

    // Handle state transitions
    switch (desiredIntakeState) {
      case HOME:
        currIntakeState = IntakeState.STOWING;
        break;
      case INTAKE:
        if (currIntakeState == IntakeState.OUTTAKE || currIntakeState == IntakeState.DEPLOYED) {
          currIntakeState = IntakeState.INTAKE;
        } else {
          currIntakeState = IntakeState.DEPLOYING;
        }
        break;
      case OUTTAKE:
        if (currIntakeState == IntakeState.INTAKE || currIntakeState == IntakeState.DEPLOYED) {
          currIntakeState = IntakeState.OUTTAKE;
        } else {
          currIntakeState = IntakeState.DEPLOYING;
        }
        break;
      case DEPLOYED:
        currIntakeState = IntakeState.DEPLOYING;
        break;
      default:
        break;
    }
  }

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
    intakeRollerIO.setMotorPercentage(INTAKE_ROLLER_DUTY_CYCLE);
  }

  public void runOuttake() {
    intakeRollerIO.setMotorPercentage(OUTTAKE_ROLLER_DUTY_CYCLE);
  }

  public void stopIntake() {
    intakeRollerIO.setMotorPercentage(0.0);
  }

  public void setIntakeArmSetpoint(double setpoint) {
    intakeArmIO.setMotorPosition(setpoint);
  }

  public void deployIntakeArm() {
    intakeArmIO.setMotorPosition(INTAKE_MOTOR_DEPLOYED_POSITION);
  }

  public void stowIntakeArm() {
    intakeArmIO.setMotorPosition(INTAKE_MOTOR_STOWED_POSITION);
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
        Math.abs(INTAKE_MOTOR_DEPLOYED_POSITION - intakeArmInputs.getMotorPosition());
    return positionError < INTAKE_ARM_POSITION_TOLERANCE;
  }

  public boolean isIntakeArmAtStowed() {
    double positionError =
        Math.abs(INTAKE_MOTOR_STOWED_POSITION - intakeArmInputs.getMotorPosition());
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

  private void handleStateTransition() {
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
        // Set intake rollers to intake speed
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
      case DEPLOYING:
        // Set intake arm to intake setpoint
        deployIntakeArm();
        // Set intake rollers off
        stopIntake();
        // When intake arm reaches setpoint, transition to INTAKE or OUTTAKE state depending on
        // desired state
        if (isIntakeArmAtDeployed()) {
          if (desiredIntakeState == IntakeState.INTAKE) {
            currIntakeState = IntakeState.INTAKE;
          } else if (desiredIntakeState == IntakeState.OUTTAKE) {
            currIntakeState = IntakeState.OUTTAKE;
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
    }
  }

  @Override
  public void periodic() {

    handleStateTransition();

    // This method will be called once per scheduler run
    intakeRollerIO.updateInputs(intakeRollerInputs);
    intakeArmIO.updateInputs(intakeArmInputs);
  }
}
