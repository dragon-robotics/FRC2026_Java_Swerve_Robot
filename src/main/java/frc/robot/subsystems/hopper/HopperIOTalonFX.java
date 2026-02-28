package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

public class HopperIOTalonFX implements HopperIO {

  protected final TalonFX motor;
  protected final int canID;
  protected final TalonFXConfiguration config;
  protected final String motorName;

  protected final VelocityTorqueCurrentFOC motorVelocityTorqueCurrentFOCRequest;

  public HopperIOTalonFX(int canID, TalonFXConfiguration config, String motorName) {
    this.canID = canID;
    this.config = config;
    this.motorName = motorName;

    motor = new TalonFX(canID);
    motor.clearStickyFaults();

    motorVelocityTorqueCurrentFOCRequest = new VelocityTorqueCurrentFOC(0);

    motor.getConfigurator().apply(config);
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
  public void updateInputs(HopperIOInputs inputs) {

    inputs.setMotorConnected(motor.isConnected());

    inputs.setMotorVoltage(motor.getMotorVoltage().getValueAsDouble());
    inputs.setMotorDutyCycle(motor.getDutyCycle().getValueAsDouble());
    inputs.setMotorCurrent(motor.getStatorCurrent().getValueAsDouble());
    inputs.setMotorTemperature(motor.getDeviceTemp().getValueAsDouble());
    inputs.setMotorVelocity(motor.getVelocity().getValueAsDouble());
  }
}
