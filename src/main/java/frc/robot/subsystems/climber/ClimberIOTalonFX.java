package frc.robot.subsystems.climber;

import static frc.robot.Constants.ClimberConstants.CLIMBER_GEAR_RATIO;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

public class ClimberIOTalonFX implements ClimberIO {

  protected final TalonFX motor;
  protected final int canID;
  protected final TalonFXConfiguration config;
  private final Optional<CANcoderConfiguration> canCoderConfig;
  private final MotionMagicExpoTorqueCurrentFOC motorMotionMagicTorqueCurrentFOC;

  public ClimberIOTalonFX(int canID, TalonFXConfiguration config) {
    this.canID = canID;
    this.config = config;
    this.canCoderConfig = Optional.empty();

    this.motor = new TalonFX(this.canID, CANBus.roboRIO());
    this.motor.clearStickyFaults();
    this.motorMotionMagicTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);
  }

  public ClimberIOTalonFX(int canID, TalonFXConfiguration config, int leadCanID,
      Optional<CANcoderConfiguration> canCoderConfig) {
    this.canID = canID;
    this.config = config;
    this.canCoderConfig = canCoderConfig;

    motor = new TalonFX(this.canID, CANBus.roboRIO());
    motor.clearStickyFaults();

    motorMotionMagicTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);
    this.canCoderConfig.ifPresentOrElse(
        ccCfg -> {
          try (CANcoder canCoder = new CANcoder(1, CANBus.roboRIO())) {
            canCoder.getConfigurator().apply(ccCfg);
            TalonFXConfiguration cfg = this.config.withFeedback(
                new FeedbackConfigs()
                    // .withFusedCANcoder(canCoder)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                    .withRotorToSensorRatio(CLIMBER_GEAR_RATIO)
                    .withSensorToMechanismRatio(1)
                    .withFeedbackRotorOffset(1)
                    .withFeedbackRotorOffset(0));
            motor.getConfigurator().apply(cfg);
          } catch (Exception e) {
            // Handle exceptions related to CANcoder configuration
            System.err.println("Error configuring CANcoder: " + e.getMessage());
          }
        },
        () -> {
          // Handle the case where canCoderConfig is not present (e.g. set up feedback
          // configs
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
  public void setMotorPosition(double setPoint) {
    motor.setControl(motorMotionMagicTorqueCurrentFOC.withPosition(setPoint));
  }

  @Override
  public void updateInputs(ClimberIOInputs climberInputs) {
    // Motor connection status
    climberInputs.setLeadMotorConnected(motor.isConnected());
    climberInputs.setFollowerMotorConnected(motor.isConnected());

    // Lead motor data
    climberInputs.setLeadMotorPosition(motor.getPosition().getValueAsDouble());
    climberInputs.setLeadMotorVelocity(motor.getVelocity().getValueAsDouble());
    climberInputs.setLeadMotorVoltage(motor.getMotorVoltage().getValueAsDouble());
    climberInputs.setLeadMotorCurrent(motor.getStatorCurrent().getValueAsDouble());
    climberInputs.setLeadMotorTemperature(motor.getDeviceTemp().getValueAsDouble());

    // Follower motor data
    climberInputs.setFollowerMotorPosition(motor.getPosition().getValueAsDouble());
    climberInputs.setFollowerMotorVelocity(motor.getVelocity().getValueAsDouble());
    climberInputs.setFollowerMotorVoltage(motor.getMotorVoltage().getValueAsDouble());
    climberInputs.setFollowerMotorCurrent(motor.getStatorCurrent().getValueAsDouble());
    climberInputs.setFollowerMotorTemperature(motor.getDeviceTemp().getValueAsDouble());
  }
}
