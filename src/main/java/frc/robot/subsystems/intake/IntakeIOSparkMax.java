package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeSubsystemConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeIOSparkMax implements IntakeIO {
  protected final SparkMax motor;
  protected final int canID;
  protected final SparkMaxConfig config;
  protected final String motorType;

  private final SparkClosedLoopController motorController;

  public IntakeIOSparkMax(int canID, SparkMaxConfig config, String motorType) {
    this.canID = canID;
    this.config = config;
    this.motorType = motorType;

    /* Instantiate the motors and encoders */
    motor = new SparkMax(this.canID, MotorType.kBrushless);
    motorController = motor.getClosedLoopController();

    /* Clear any existing faults */
    motor.clearFaults();

    motor.configure(this.config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setMotorVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setMotorPercentage(double percentage) {
    motor.set(percentage);
  }

  @Override
  public void setMotorRPM(double rpm) {
    motorController.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void setMotorPosition(double setpoint) {
    motorController.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    inputs.setMotorConnected(motor.getDeviceId() == canID);

    inputs.setMotorVoltage(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.setMotorDutyCycle(motor.getAppliedOutput());
    inputs.setMotorCurrent(motor.getOutputCurrent());
    inputs.setMotorTemperature(motor.getMotorTemperature());
    inputs.setMotorVelocity(motor.getAlternateEncoder().getVelocity());
    inputs.setMotorPosition(motor.getAbsoluteEncoder().getPosition());
  }
}
