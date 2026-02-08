package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeSubsystemConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX motor;

  private final MotionMagicVelocityTorqueCurrentFOC motorMotionMagicVelocityTorqueCurrentFOCRequest;
  private final MotionMagicExpoTorqueCurrentFOC motorMotionMagicExpoTorqueCurrentFOCRequest;

  public IntakeIOTalonFX(int canID, TalonFXConfiguration config) {

    /* Instantiate the motors */
    motor = new TalonFX(canID, CANBus.roboRIO());

    /* Clear any existing faults */
    motor.clearStickyFaults();

    /* Configure the motors */
    motor.getConfigurator().apply(config);

    /* Create Motion Magic Velocity and Motion Magic Expo requests */
    motorMotionMagicVelocityTorqueCurrentFOCRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
    motorMotionMagicExpoTorqueCurrentFOCRequest = new MotionMagicExpoTorqueCurrentFOC(0);
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
