package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;

public class ShooterIOTalonFX implements ShooterIO {
  protected final TalonFX motor;
  protected final int canID;
  protected final TalonFXConfiguration config;

  protected final VelocityTorqueCurrentFOC motorVelocityTorqueCurrentFOCRequest;

  public ShooterIOTalonFX(int canID, TalonFXConfiguration config) {
    this.canID = canID;
    this.config = config;

    motor = new TalonFX(canID);
    motor.clearStickyFaults();

    motorVelocityTorqueCurrentFOCRequest = new VelocityTorqueCurrentFOC(0);

    motor.getConfigurator().apply(config);
  }

  public ShooterIOTalonFX(int canID, TalonFXConfiguration config, int leadCANID) {
    this.canID = canID;
    this.config = config;

    motor = new TalonFX(canID);
    motor.clearStickyFaults();

    motorVelocityTorqueCurrentFOCRequest = new VelocityTorqueCurrentFOC(0);
    motor.getConfigurator().apply(config);
    motor.setControl(new Follower(leadCANID, MotorAlignmentValue.Opposed));
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
  public void updateInputs(ShooterIOInputs inputs) {

    inputs.setMotorConnected(motor.isConnected());

    inputs.setMotorVoltage(motor.getMotorVoltage().getValueAsDouble());
    inputs.setMotorDutyCycle(motor.getDutyCycle().getValueAsDouble());
    inputs.setMotorCurrent(motor.getStatorCurrent().getValueAsDouble());
    inputs.setMotorTemperature(motor.getDeviceTemp().getValueAsDouble());
    inputs.setMotorVelocity(motor.getVelocity().getValueAsDouble());
  }  
}