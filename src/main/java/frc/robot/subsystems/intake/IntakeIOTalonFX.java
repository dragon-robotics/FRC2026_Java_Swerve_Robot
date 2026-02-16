package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeSubsystemConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import java.util.Optional;

public class IntakeIOTalonFX implements IntakeIO {

  protected final TalonFX motor;
  protected final int canID;
  protected final TalonFXConfiguration config;

  // Position Controls //
  protected final PositionVoltage motorPositionVoltageRequest;
  protected final PositionTorqueCurrentFOC motorPositionTorqueCurrentFOCRequest;
  protected final MotionMagicVoltage motorMotionMagicVoltageRequest;
  protected final MotionMagicTorqueCurrentFOC motorMotionMagicTorqueCurrentFOCRequest;
  protected final MotionMagicExpoVoltage motorMotionMagicExpoVoltageRequest;
  protected final MotionMagicExpoTorqueCurrentFOC motorMotionMagicExpoTorqueCurrentFOCRequest;

  // Velocity Controls //
  protected final MotionMagicVelocityVoltage motorMotionMagicVelocityVoltageRequest;
  protected final MotionMagicVelocityTorqueCurrentFOC
      motorMotionMagicVelocityTorqueCurrentFOCRequest;
  protected final VelocityVoltage motorVelocityVoltageRequest;
  protected final VelocityTorqueCurrentFOC motorVelocityTorqueCurrentFOCRequest;

  private final Optional<CANcoderConfiguration> canCoderConfig;

  public IntakeIOTalonFX(int canID, TalonFXConfiguration config) {
    this(canID, config, Optional.empty());
  }

  public IntakeIOTalonFX(
      int canID, TalonFXConfiguration config, Optional<CANcoderConfiguration> canCoderConfig) {
    this.canID = canID;
    this.config = config;
    this.canCoderConfig = canCoderConfig;

    motor = new TalonFX(this.canID, CANBus.roboRIO());
    motor.clearStickyFaults();

    /* Create Position and Velocity requests */

    // Position Control Requests
    motorPositionVoltageRequest = new PositionVoltage(0);
    motorPositionTorqueCurrentFOCRequest = new PositionTorqueCurrentFOC(0);
    motorMotionMagicVoltageRequest = new MotionMagicVoltage(0);
    motorMotionMagicTorqueCurrentFOCRequest = new MotionMagicTorqueCurrentFOC(0);
    motorMotionMagicExpoVoltageRequest = new MotionMagicExpoVoltage(0);
    motorMotionMagicExpoTorqueCurrentFOCRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    // Velocity Control Requests
    motorVelocityVoltageRequest = new VelocityVoltage(0);
    motorVelocityTorqueCurrentFOCRequest = new VelocityTorqueCurrentFOC(0);
    motorMotionMagicVelocityVoltageRequest = new MotionMagicVelocityVoltage(0);
    motorMotionMagicVelocityTorqueCurrentFOCRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

    // Apply CANcoder config (absolute offset/direction) if cancoder config is present
    this.canCoderConfig.ifPresentOrElse(
        ccCfg -> {
          try (CANcoder canCoder = new CANcoder(INTAKE_ARM_CANCODER_ID, CANBus.roboRIO())) {
            canCoder.getConfigurator().apply(ccCfg);
          }

          TalonFXConfiguration cfg =
              this.config.withFeedback(
                  new FeedbackConfigs()
                      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                      .withRotorToSensorRatio(INTAKE_ARM_GEAR_RATIO)
                      .withSensorToMechanismRatio(1)
                      .withFeedbackRotorOffset(0));

          motor.getConfigurator().apply(cfg);
        },
        () -> {
          // Handle the case where canCoderConfig is not present (e.g. set up feedback configs
          // without CANcoder)
          motor.getConfigurator().apply(this.config);
          motor.setPosition(0);
        });
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
    motor.setControl(motorMotionMagicVelocityTorqueCurrentFOCRequest.withVelocity(rpm / 60));
  }

  @Override
  public void setMotorPosition(double setpoint) {
    motor.setControl(motorMotionMagicExpoTorqueCurrentFOCRequest.withPosition(setpoint));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    inputs.setMotorConnected(motor.isConnected());

    inputs.setMotorVoltage(motor.getMotorVoltage().getValueAsDouble());
    inputs.setMotorDutyCycle(motor.getDutyCycle().getValueAsDouble());
    inputs.setMotorCurrent(motor.getStatorCurrent().getValueAsDouble());
    inputs.setMotorTemperature(motor.getDeviceTemp().getValueAsDouble());
    inputs.setMotorVelocity(motor.getVelocity().getValueAsDouble());
    inputs.setMotorPosition(motor.getPosition().getValueAsDouble());
  }
}
