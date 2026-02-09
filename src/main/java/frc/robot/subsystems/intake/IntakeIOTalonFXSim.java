package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeIOTalonFXSim extends IntakeIOTalonFX {

  private final DCMotor motorType;

  public IntakeIOTalonFXSim(int canID, TalonFXConfiguration config, String motorType) {
    super(canID, config, motorType);

    switch (motorType) {
      case "NEO":
        this.motorType = DCMotor.getNEO(1);
        break;
      case "NEO550":
        this.motorType = DCMotor.getNeo550(1);
        break;
      case "Vortex":
        this.motorType = DCMotor.getNeoVortex(1);
        break;
      default:
        this.motorType = DCMotor.getNEO(1);
    }
  }

  public TalonFXSimState getSimState() {
    return motor.getSimState();
  }

  public DCMotor getMotorType() {
    return motorType;
  }
}
