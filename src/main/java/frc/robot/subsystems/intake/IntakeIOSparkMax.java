package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.Optional;

public class IntakeIOSparkMax implements IntakeIO {

  protected enum EncoderMode {
    PRIMARY,
    ABSOLUTE,
    ALTERNATE
  }

  protected final SparkMax motor;
  protected final int canID;
  protected final SparkMaxConfig config;
  protected final String motorType;

  private final SparkClosedLoopController motorController;
  private final EncoderMode encoderMode;

  public IntakeIOSparkMax(int canID, SparkMaxConfig config, String motorType, String motorName) {
    this(canID, config, motorType, motorName, EncoderMode.PRIMARY, Optional.empty(), Optional.empty());
  }

  public IntakeIOSparkMax(
      int canID, SparkMaxConfig config, String motorType, String motorName, AbsoluteEncoderConfig absEncoderConfig) {
    this(
        canID,
        config,
        motorType,
        motorName,
        EncoderMode.ABSOLUTE,
        Optional.of(absEncoderConfig),
        Optional.empty());
  }

  public IntakeIOSparkMax(
      int canID, SparkMaxConfig config, String motorType, String motorName, AlternateEncoderConfig altEncoderConfig) {
    this(
        canID,
        config,
        motorType,
        motorName,
        EncoderMode.ALTERNATE,
        Optional.empty(),
        Optional.of(altEncoderConfig));
  }

  public IntakeIOSparkMax(
      int canID,
      SparkMaxConfig config,
      String motorType,
      String motorName,
      EncoderMode encoderMode,
      Optional<AbsoluteEncoderConfig> absEncoderConfig,
      Optional<AlternateEncoderConfig> altEncoderConfig) {
    this.canID = canID;
    this.config = config;
    this.motorType = motorType;
    this.encoderMode = encoderMode;

    /* Instantiate the motors and encoders */
    motor = new SparkMax(this.canID, MotorType.kBrushless);
    motorController = motor.getClosedLoopController();

    /* Clear any existing faults */
    motor.clearFaults();

    absEncoderConfig.ifPresent(this.config::apply);
    altEncoderConfig.ifPresent(this.config::apply);

    motor.configure(this.config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(0);
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
    inputs.setMotorVelocity(motor.getEncoder().getVelocity());

    switch (encoderMode) {
      case ABSOLUTE -> inputs.setMotorPosition(motor.getAbsoluteEncoder().getPosition());
      case ALTERNATE -> inputs.setMotorPosition(motor.getAlternateEncoder().getPosition());
      case PRIMARY -> inputs.setMotorPosition(motor.getEncoder().getPosition());
    }
  }
}
