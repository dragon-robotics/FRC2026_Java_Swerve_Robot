package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

public class HopperTalonFX implements HopperIO {

  private final TalonFX hopperMotor;

  private final MotionMagicVelocityTorqueCurrentFOC rollerMotionMagicVelocityTorqueCurrentFOC;
  private final MotionMagicExpoTorqueCurrentFOC expandingMotorMotionMagicTorqueCurrentFOC;

  public HopperTalonFX(int canID, TalonFXConfiguration config) {
    hopperMotor = new TalonFX(canID);
    hopperMotor.getConfigurator().apply(config);

    rollerMotionMagicVelocityTorqueCurrentFOC = new MotionMagicVelocityTorqueCurrentFOC(0);
    expandingMotorMotionMagicTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);
  }

  @Override
  public void expandHopper(double setpoint) {
    hopperMotor.setControl(expandingMotorMotionMagicTorqueCurrentFOC.withPosition(setpoint));
  }

  @Override
  public void runRollers(double rpm) {
    hopperMotor.setControl(rollerMotionMagicVelocityTorqueCurrentFOC.withVelocity(rpm));
  }

  @Override
  public void stowHopper() {
    expandHopper(0.0);
    runRollers(0.0);
  }

  @Override
  public void updateInputs(HopperIOInputs hopperInputs) {
    // Motor connection status
    hopperInputs.setExpandingMotorConnected(hopperMotor.isConnected());

    // Expanding motor data
    hopperInputs.setExpandingMotorPosition(hopperMotor.getPosition().getValueAsDouble());
    hopperInputs.setExpandingMotorVelocity(hopperMotor.getVelocity().getValueAsDouble());
    hopperInputs.setExpandingMotorVoltage(hopperMotor.getMotorVoltage().getValueAsDouble());
    hopperInputs.setExpandingMotorTemprature(hopperMotor.getDeviceTemp().getValueAsDouble());
    hopperInputs.setExpandingMotorCurrent(hopperMotor.getStatorCurrent().getValueAsDouble());
  }
}
