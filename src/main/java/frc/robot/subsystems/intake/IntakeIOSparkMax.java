package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeSubsystemConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax intakeRollerMotor;
  private final SparkMax intakeArmMotor;

  private final SparkClosedLoopController rollerController;
  private final SparkMaxAlternateEncoder rollerAlternateEncoder;
  private final SparkClosedLoopController armController;
  private final SparkAbsoluteEncoder armAbsoluteEncoder;

  public IntakeIOSparkMax() {
    /* Instantiate the motors and encoders */
    intakeRollerMotor = new SparkMax(INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);
    rollerController = intakeRollerMotor.getClosedLoopController();
    rollerAlternateEncoder = (SparkMaxAlternateEncoder) intakeRollerMotor.getAlternateEncoder();

    intakeArmMotor = new SparkMax(INTAKE_ARM_MOTOR_ID, MotorType.kBrushless);
    armController = intakeArmMotor.getClosedLoopController();
    armAbsoluteEncoder = intakeArmMotor.getAbsoluteEncoder();

    /* Clear any existing faults */
    intakeRollerMotor.clearFaults();
    intakeArmMotor.clearFaults();

    /* Configure the motors */
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    SparkMaxConfig armConfig = new SparkMaxConfig();

    /* Roller motor configuration */
    rollerConfig
        .voltageCompensation(INTAKE_ARM_MAX_VOLTAGE)
        .smartCurrentLimit(
            (int) INTAKE_ROLLER_STATOR_CURRENT_LIMIT, (int) INTAKE_ROLLER_STATOR_CURRENT_LIMIT)
        .secondaryCurrentLimit(INTAKE_ARM_SUPPLY_CURRENT_LIMIT)
        .openLoopRampRate(INTAKE_ROLLER_RAMP_RATE)
        .closedLoopRampRate(INTAKE_ROLLER_RAMP_RATE)
        .idleMode(IdleMode.kCoast);

    /* Roller motor PID configs for Velocity Control */
    rollerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1)
        .maxMotion
        .cruiseVelocity(INTAKE_ROLLER_RPM, ClosedLoopSlot.kSlot0)
        .maxAcceleration(INTAKE_ROLLER_RPM * 2, ClosedLoopSlot.kSlot0)
        .allowedProfileError(INTAKE_ROLLER_RPM * 0.01, ClosedLoopSlot.kSlot0);

    /* Arm motor configuration */
    armConfig
        .voltageCompensation(INTAKE_ARM_MAX_VOLTAGE)
        .smartCurrentLimit(
            (int) INTAKE_ARM_STATOR_CURRENT_LIMIT, (int) INTAKE_ARM_STATOR_CURRENT_LIMIT)
        .secondaryCurrentLimit(INTAKE_ARM_SUPPLY_CURRENT_LIMIT)
        .openLoopRampRate(INTAKE_ARM_RAMP_RATE)
        .closedLoopRampRate(INTAKE_ARM_RAMP_RATE)
        .idleMode(IdleMode.kBrake);

    /* Arm motor PID configs for Position Control */
    armConfig.absoluteEncoder.zeroOffset(0).inverted(false);

    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1)
        .maxMotion
        .cruiseVelocity(INTAKE_ARM_MAXMOTION_MAX_VELOCITY, ClosedLoopSlot.kSlot0)
        .maxAcceleration(INTAKE_ARM_MAXMOTION_MAX_ACCELERATION, ClosedLoopSlot.kSlot0)
        .allowedProfileError(0.05, ClosedLoopSlot.kSlot0)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);

    intakeRollerMotor.configure(
        rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    intakeArmMotor.configure(
        armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setIntakeRollerMotorVoltage(double voltage) {
    intakeRollerMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeRollerMotorPercentage(double percentage) {
    intakeRollerMotor.set(percentage);
  }

  @Override
  public void setIntakeRollerMotorRPM(double rpm) {
    rollerController.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void setIntakeArmMotorVoltage(double voltage) {
    intakeArmMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeArmMotorPercentage(double percentage) {
    intakeArmMotor.set(percentage);
  }

  @Override
  public void setIntakeArmMotorSetpoint(double setpoint) {
    armController.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    inputs.setIntakeRollerMotorConnected(intakeRollerMotor.getDeviceId() == INTAKE_ROLLER_MOTOR_ID);
    inputs.setIntakeArmMotorConnected(intakeArmMotor.getDeviceId() == INTAKE_ARM_MOTOR_ID);

    inputs.setIntakeRollerMotorVoltage(
        intakeRollerMotor.getAppliedOutput() * intakeRollerMotor.getBusVoltage());
    inputs.setIntakeRollerMotorDutyCycle(intakeRollerMotor.getAppliedOutput());
    inputs.setIntakeRollerMotorCurrent(intakeRollerMotor.getOutputCurrent());
    inputs.setIntakeRollerMotorTemperature(intakeRollerMotor.getMotorTemperature());
    inputs.setIntakeRollerMotorVelocity(intakeRollerMotor.getAlternateEncoder().getVelocity());

    inputs.setIntakeArmMotorVoltage(
        intakeArmMotor.getAppliedOutput() * intakeArmMotor.getBusVoltage());
    inputs.setIntakeArmMotorDutyCycle(intakeArmMotor.getAppliedOutput());
    inputs.setIntakeArmMotorCurrent(intakeArmMotor.getOutputCurrent());
    inputs.setIntakeArmMotorTemperature(intakeArmMotor.getMotorTemperature());
    inputs.setIntakeArmMotorSetpoint(intakeArmMotor.getAbsoluteEncoder().getPosition());
  }
}
