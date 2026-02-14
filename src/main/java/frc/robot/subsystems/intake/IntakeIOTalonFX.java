package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeSubsystemConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import java.util.Optional;

public class IntakeIOTalonFX implements IntakeIO {

  protected final TalonFX motor;
  protected final int canID;
  protected final TalonFXConfiguration config;

  protected final VelocityTorqueCurrentFOC
      motorVelocityTorqueCurrentFOCRequest;
  protected final MotionMagicExpoTorqueCurrentFOC motorMotionMagicExpoTorqueCurrentFOCRequest;

  private final Optional<CANcoderConfiguration> canCoderConfig;

  public IntakeIOTalonFX(int canID, TalonFXConfiguration config) {
    this(canID, config, Optional.empty());
  }

  public IntakeIOTalonFX(
      int canID,
      TalonFXConfiguration config,
      Optional<CANcoderConfiguration> canCoderConfig) {
    this.canID = canID;
    this.config = config;
    this.canCoderConfig = canCoderConfig;

    motor = new TalonFX(this.canID, CANBus.roboRIO());
    motor.clearStickyFaults();

    /* Create Motion Magic Velocity and Motion Magic Expo requests */
    motorVelocityTorqueCurrentFOCRequest = new VelocityTorqueCurrentFOC(0);
    motorMotionMagicExpoTorqueCurrentFOCRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    // Apply CANcoder config (absolute offset/direction) if cancoder config is present
    this.canCoderConfig.ifPresentOrElse(
        ccCfg -> {
          try (CANcoder canCoder = new CANcoder(INTAKE_ARM_CANCODER_ID, CANBus.roboRIO())) {
            canCoder.getConfigurator().apply(ccCfg);
            TalonFXConfiguration cfg =
                this.config.withFeedback(
                    new FeedbackConfigs()
                        // .withFusedCANcoder(canCoder)
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                        .withRotorToSensorRatio(INTAKE_ARM_GEAR_RATIO)
                        .withSensorToMechanismRatio(1)
                        .withFeedbackRotorOffset(0));
            motor.getConfigurator().apply(cfg);
          } catch (Exception e) {
            // Handle exceptions related to CANcoder configuration
            System.err.println("Error configuring CANcoder: " + e.getMessage());
          }
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
    motor.setControl(motorVelocityTorqueCurrentFOCRequest.withVelocity(rpm / 60));
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
