package frc.robot.io;

import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_DEPLOYED_POSITION;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;

public class TalonFXMotorIO implements MotorIO {

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

  // Cached status signals — refreshed in batch to optimize loop time and use
  // opt-in to optimize CAN
  // bus util
  protected final StatusSignal<Voltage> motorVoltageSignal;
  protected final StatusSignal<Current> statorCurrentSignal;
  protected final StatusSignal<Temperature> deviceTempSignal;
  protected final StatusSignal<AngularVelocity> velocitySignal;
  protected final StatusSignal<Angle> positionSignal;

  // Needs to be enabled for follower motors
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
    positionVoltageRequest = new PositionVoltage(INTAKE_ARM_DEPLOYED_POSITION);
    dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);
    voltageRequest = new VoltageOut(0).withEnableFOC(true);

    // Disabled all unused signals first
    motor.optimizeBusUtilization(0);

    // Initialize status signals that we are using
    motorVoltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    deviceTempSignal = motor.getDeviceTemp();
    velocitySignal = motor.getVelocity();
    positionSignal = motor.getPosition();

    dutyCycleSignal = motor.getDutyCycle();
    torqueCurrentSignal = motor.getTorqueCurrent();

    // Re-enable update for signals that we are using
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        dutyCycleSignal, // required for Follower
        motorVoltageSignal, // required for Follower
        torqueCurrentSignal, // required for Follower
        statorCurrentSignal, // fault detection
        velocitySignal, // closed-loop feedback
        positionSignal); // closed-loop feedback

    deviceTempSignal.setUpdateFrequency(
        0.25); // temperature doesn't need to be updated as often, so we set it to 0.25Hz or
               // every 4
    // seconds

    // Register the motor to the signal registry
    SignalRegistry.getInstance().registerMotorIO(this);

    // Apply CANcoder config if present
    canCoderConfig.ifPresent(
        ccCfg -> {
          canCoder = new CANcoder(canID);
          canCoder.getConfigurator().apply(ccCfg);
          TalonFXConfiguration updatedConfig = config.withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(canCoder.getDeviceID())
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));
          motor.getConfigurator().apply(updatedConfig);

          canCoder.optimizeBusUtilization(0);

          // Modify CANCoder signal frequencies to match the motor's closed-loop update
          // rate for
          // better synchronization
          BaseStatusSignal.setUpdateFrequencyForAll(
              100, canCoder.getPosition(), canCoder.getVelocity(), canCoder.getAbsolutePosition());
        });

    if (canCoderConfig.isEmpty()) {
      motor.getConfigurator().apply(config);
    }

    // Apply follower config if present
    followerConfig.ifPresent(motor::setControl);
  }

  @Override
  public String getMotorName() {
    return motorName;
  }

  @Override
  public void setMotorVoltage(double voltage) {
    motor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setMotorPercentage(double percentage) {
    motor.setControl(dutyCycleRequest.withOutput(percentage));
  }

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

  /** Expose for sim subclasses. */
  public TalonFX getMotor() {
    return motor;
  }
}
