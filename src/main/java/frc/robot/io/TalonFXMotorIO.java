package frc.robot.io;

import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_DEPLOYED_POSITION;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;

/**
 * CTRE Talon FX implementation of {@link MotorIO}.
 *
 * <p>Velocity commands are accepted in RPM and converted to rotations per second for Phoenix.
 * Position commands use mechanism rotations. Status signals are registered with {@link
 * SignalRegistry} for batched refresh before subsystem inputs are read.
 */
public class TalonFXMotorIO implements TorqueCurrentMotorIO {
  private static final double CONTROL_SIGNAL_FREQUENCY_HZ = 100.0;
  private static final double TEMPERATURE_SIGNAL_FREQUENCY_HZ = 0.25;

  protected final TalonFX motor;
  protected final int canID;
  protected final TalonFXConfiguration config;
  protected final String motorName;
  protected CANcoder canCoder;

  protected final VelocityTorqueCurrentFOC velocityRequest;
  protected final MotionMagicVelocityTorqueCurrentFOC mmVelocityRequest;
  protected final MotionMagicExpoTorqueCurrentFOC mmPositionRequest;
  protected final PositionVoltage positionVoltageRequest;
  protected final DutyCycleOut dutyCycleRequest;
  protected final VoltageOut voltageRequest;
  protected final TorqueCurrentFOC torqueCurrentRequest;

  /** Cached status signals refreshed in one batch call to reduce CAN bus load. */
  protected final StatusSignal<Voltage> motorVoltageSignal;

  protected final StatusSignal<Current> statorCurrentSignal;
  protected final StatusSignal<Temperature> deviceTempSignal;
  protected final StatusSignal<AngularVelocity> velocitySignal;
  protected final StatusSignal<Angle> positionSignal;

  /** Signals needed for follower monitoring and torque-current telemetry. */
  protected final StatusSignal<Double> dutyCycleSignal;

  protected final StatusSignal<Current> torqueCurrentSignal;

  /** Standard motor (no follower, no CANcoder). */
  public TalonFXMotorIO(int canID, TalonFXConfiguration config, String motorName) {
    this(canID, config, motorName, Optional.empty(), Optional.empty());
  }

  /** Follower motor. */
  public TalonFXMotorIO(
      int canID, TalonFXConfiguration config, String motorName, Follower followerConfig) {
    this(canID, config, motorName, Optional.empty(), Optional.of(followerConfig));
  }

  /** Motor with external CANcoder feedback. */
  public TalonFXMotorIO(
      int canID,
      TalonFXConfiguration config,
      String motorName,
      CANcoderConfiguration canCoderConfig) {
    this(canID, config, motorName, Optional.of(canCoderConfig), Optional.empty());
  }

  /** Full constructor. */
  public TalonFXMotorIO(
      int canID,
      TalonFXConfiguration config,
      String motorName,
      Optional<CANcoderConfiguration> canCoderConfig,
      Optional<Follower> followerConfig) {
    this.canID = canID;
    this.config = config;
    this.motorName = motorName;

    motor = new TalonFX(canID);
    motor.clearStickyFaults();

    velocityRequest = new VelocityTorqueCurrentFOC(0);
    mmVelocityRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
    mmPositionRequest = new MotionMagicExpoTorqueCurrentFOC(0);
    positionVoltageRequest = new PositionVoltage(INTAKE_ARM_DEPLOYED_POSITION).withEnableFOC(true);
    dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);
    voltageRequest = new VoltageOut(0).withEnableFOC(true);
    torqueCurrentRequest = new TorqueCurrentFOC(0);

    motor.optimizeBusUtilization(0);

    motorVoltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    deviceTempSignal = motor.getDeviceTemp();
    velocitySignal = motor.getVelocity();
    positionSignal = motor.getPosition();

    dutyCycleSignal = motor.getDutyCycle();
    torqueCurrentSignal = motor.getTorqueCurrent();

    configureStatusSignalFrequencies();
    SignalRegistry.getInstance().registerMotorIO(this);
    configureRemoteCancoder(canCoderConfig);

    if (canCoderConfig.isEmpty()) {
      motor.getConfigurator().apply(config);
    }

    followerConfig.ifPresent(motor::setControl);
  }

  private void configureStatusSignalFrequencies() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        CONTROL_SIGNAL_FREQUENCY_HZ,
        dutyCycleSignal,
        motorVoltageSignal,
        torqueCurrentSignal,
        statorCurrentSignal,
        velocitySignal,
        positionSignal);

    deviceTempSignal.setUpdateFrequency(TEMPERATURE_SIGNAL_FREQUENCY_HZ);
  }

  private void configureRemoteCancoder(Optional<CANcoderConfiguration> canCoderConfig) {
    canCoderConfig.ifPresent(
        ccCfg -> {
          canCoder = new CANcoder(canID);
          canCoder.getConfigurator().apply(ccCfg);
          TalonFXConfiguration updatedConfig =
              config.withFeedback(
                  new FeedbackConfigs()
                      .withFeedbackRemoteSensorID(canCoder.getDeviceID())
                      .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));
          motor.getConfigurator().apply(updatedConfig);

          canCoder.optimizeBusUtilization(0);

          BaseStatusSignal.setUpdateFrequencyForAll(
              CONTROL_SIGNAL_FREQUENCY_HZ,
              canCoder.getPosition(),
              canCoder.getVelocity(),
              canCoder.getAbsolutePosition());
        });
  }

  @Override
  public String getMotorName() {
    return motorName;
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    motor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setMotorPercentage(double percentage) {
    motor.setControl(dutyCycleRequest.withOutput(percentage));
  }

  /** Converts RPM to rotations per second for Phoenix velocity closed loop. */
  @Override
  public void setMotorRPM(double rpm) {
    motor.setControl(velocityRequest.withVelocity(rpm / 60.0));
  }

  @Override
  public void setMotorPosition(double setpoint) {
    motor.setControl(positionVoltageRequest.withPosition(setpoint));
  }

  @Override
  public void setMotorPosition(double setpoint, int slotID) {
    motor.setControl(positionVoltageRequest.withPosition(setpoint).withSlot(slotID));
  }

  @Override
  public void setMotorTorqueCurrent(Current torqueCurrent) {
    motor.setControl(torqueCurrentRequest.withOutput(torqueCurrent));
  }

  @Override
  public BaseStatusSignal[] getStatusSignals() {
    return new BaseStatusSignal[] {
      motorVoltageSignal,
      statorCurrentSignal,
      deviceTempSignal,
      velocitySignal,
      positionSignal,
      dutyCycleSignal,
      torqueCurrentSignal
    };
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    inputs.setMotorConnected(
        BaseStatusSignal.isAllGood(
            motorVoltageSignal, statorCurrentSignal, velocitySignal, positionSignal));
    inputs.setMotorVoltage(motorVoltageSignal.getValueAsDouble());
    inputs.setMotorCurrent(statorCurrentSignal.getValueAsDouble());
    inputs.setMotorTemperature(deviceTempSignal.getValueAsDouble());
    inputs.setMotorVelocity(velocitySignal.getValueAsDouble());
    inputs.setMotorPosition(positionSignal.getValueAsDouble());
  }

  /** Returns the backing TalonFX for simulation wrappers and tuning helpers. */
  public TalonFX getMotor() {
    return motor;
  }
}
