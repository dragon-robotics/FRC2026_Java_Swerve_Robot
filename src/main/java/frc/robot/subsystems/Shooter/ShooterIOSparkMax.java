package frc.robot.subsystems.shooter;

import java.util.Optional;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOSparkMax implements ShooterIO {

  protected final SparkMax motor;
  protected final int canID;
  protected final SparkMaxConfig config;
  protected final String motorType;

  private final SparkClosedLoopController motorController;
  private final Optional<AlternateEncoderConfig> altEncoderConfig;

    public ShooterIOSparkMax(
        int canID,
        SparkMaxConfig config,
        String motorType) {
      this(canID, config, motorType, Optional.empty());
    }
    
   public ShooterIOSparkMax(
        int canID, SparkMaxConfig config, String motorType, Optional<AlternateEncoderConfig> altEncoderConfig) {
      this.canID = canID;
      this.config = config;
      this.motorType = motorType;
      this.altEncoderConfig = altEncoderConfig;

      motor = new SparkMax(canID, MotorType.kBrushless);
      motorController = motor.getClosedLoopController();

      motor.clearFaults();

      this.altEncoderConfig.ifPresentOrElse(
          altEncCfg -> {
            this.config.apply(altEncCfg);
            motor.configure(
                this.config, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
            motor.getEncoder().setPosition(0);
          },
          () -> {
            // Handle the case where altEncoderConfig is not present
            motor.configure(
                this.config, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
            motor.getEncoder().setPosition(0);
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
    motorController.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
  }
  
  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    inputs.setMotorConnected(motor.getDeviceId() == canID);

    inputs.setMotorVoltage(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.setMotorDutyCycle(motor.getAppliedOutput());
    inputs.setMotorCurrent(motor.getOutputCurrent());
    inputs.setMotorTemperature(motor.getMotorTemperature());
    inputs.setMotorVelocity(motor.getEncoder().getVelocity());
  }  
    
}
