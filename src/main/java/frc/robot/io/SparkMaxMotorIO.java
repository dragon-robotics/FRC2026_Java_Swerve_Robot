package frc.robot.io;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;

/**
 * REV Spark MAX implementation of {@link MotorIO}.
 *
 * <p>Velocity setpoints use RPM. Position setpoints use the configured encoder's position units.
 * Spark MAX does not support the torque-current control mode, so that capability lives in
 * {@link TorqueCurrentMotorIO} instead of this class.
 */
public class SparkMaxMotorIO implements MotorIO {
  /** Encoder source used for position telemetry. */
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

  /** Creates a Spark MAX using the primary relative encoder. */
  public SparkMaxMotorIO(int canID, SparkMaxConfig config, String motorType, String motorName) {
    this(
        canID,
        config,
        motorType,
        motorName,
        EncoderMode.PRIMARY,
        Optional.empty(),
        Optional.empty());
  }

  /** Creates a Spark MAX using the absolute encoder. */
  public SparkMaxMotorIO(
      int canID,
      SparkMaxConfig config,
      String motorType,
      String motorName,
      AbsoluteEncoderConfig absEncoderConfig) {
    this(
        canID,
        config,
        motorType,
        motorName,
        EncoderMode.ABSOLUTE,
        Optional.of(absEncoderConfig),
        Optional.empty());
  }

  /** Creates a Spark MAX using the alternate encoder. */
  public SparkMaxMotorIO(
      int canID,
      SparkMaxConfig config,
      String motorType,
      String motorName,
      AlternateEncoderConfig altEncoderConfig) {
    this(
        canID,
        config,
        motorType,
        motorName,
        EncoderMode.ALTERNATE,
        Optional.empty(),
        Optional.of(altEncoderConfig));
  }

  /** Full constructor for selecting the encoder source and optional encoder config. */
  public SparkMaxMotorIO(
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

    motor = new SparkMax(this.canID, MotorType.kBrushless);
    motorController = motor.getClosedLoopController();

    motor.clearFaults();

    applyEncoderConfigs(absEncoderConfig, altEncoderConfig);

    motor.configure(this.config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(0);
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
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
  public void setMotorPosition(double setpoint, int slotID) {
    motorController.setSetpoint(
        setpoint, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.values()[slotID]);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    inputs.setMotorConnected(motor.getDeviceId() == canID);
    inputs.setMotorVoltage(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.setMotorCurrent(motor.getOutputCurrent());
    inputs.setMotorTemperature(motor.getMotorTemperature());
    inputs.setMotorVelocity(motor.getEncoder().getVelocity());
    inputs.setMotorPosition(readSelectedEncoderPosition());
  }

  private void applyEncoderConfigs(
      Optional<AbsoluteEncoderConfig> absEncoderConfig,
      Optional<AlternateEncoderConfig> altEncoderConfig) {
    absEncoderConfig.ifPresent(this.config::apply);
    altEncoderConfig.ifPresent(this.config::apply);
  }

  private double readSelectedEncoderPosition() {
    return switch (encoderMode) {
      case ABSOLUTE -> motor.getAbsoluteEncoder().getPosition();
      case ALTERNATE -> motor.getAlternateEncoder().getPosition();
      case PRIMARY -> motor.getEncoder().getPosition();
    };
  }
}
