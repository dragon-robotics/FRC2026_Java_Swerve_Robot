package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOTalonFX implements ShooterIO {
  protected final TalonFX motor;
  protected final int canID;
  protected final TalonFXConfiguration config;

  protected final MotionMagicExpoVoltage motorMotionMagicExpoVoltageRequest;

  protected final VelocityVoltage motorVelocityVoltageRequest;
  protected final VelocityTorqueCurrentFOC motorVelocityTorqueCurrentFOCRequest;
  protected final MotionMagicVelocityVoltage motorMotionMagicVelocityVoltageRequest;
  protected final MotionMagicVelocityTorqueCurrentFOC
      motorMotionMagicVelocityTorqueCurrentFOCRequest;

  public ShooterIOTalonFX(
      int canID, TalonFXConfiguration config, String motorName, Follower followCfg) {
    this(canID, config, motorName); // reuse common init
    motor.setControl(followCfg);
  }

  public ShooterIOTalonFX(int canID, TalonFXConfiguration config, String motorName) {
    this.canID = canID;
    this.config = config;

    motor = new TalonFX(canID);
    motor.clearStickyFaults();

    motorMotionMagicExpoVoltageRequest = new MotionMagicExpoVoltage(0);

    motorVelocityVoltageRequest = new VelocityVoltage(0);
    motorVelocityTorqueCurrentFOCRequest = new VelocityTorqueCurrentFOC(0);
    motorMotionMagicVelocityVoltageRequest = new MotionMagicVelocityVoltage(0);
    motorMotionMagicVelocityTorqueCurrentFOCRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

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
  public void setMotorPosition(double position) {
    motor.setControl(motorMotionMagicExpoVoltageRequest.withPosition(position));
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
