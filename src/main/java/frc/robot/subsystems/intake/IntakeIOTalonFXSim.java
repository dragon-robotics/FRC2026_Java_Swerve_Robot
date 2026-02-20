package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Optional;

public class IntakeIOTalonFXSim extends IntakeIOTalonFX {

  private final DCMotor motorType;

  public IntakeIOTalonFXSim(
      int canID, TalonFXConfiguration config, String motorType, String motorName) {
    this(canID, config, motorType, motorName, Optional.empty());
  }

  public IntakeIOTalonFXSim(
      int canID,
      TalonFXConfiguration config,
      String motorType,
      String motorName,
      Optional<CANcoderConfiguration> canCoderConfig) {
    super(canID, config, motorName, canCoderConfig);

    switch (motorType) {
      case "KrakenX60":
        this.motorType = DCMotor.getKrakenX60(1);
        break;
      case "KrakenX60_FOC":
        this.motorType = DCMotor.getKrakenX60Foc(1);
        break;
      case "KrakenX44":
        this.motorType = DCMotor.getKrakenX44(1);
        break;
      case "KrakenX44_FOC":
        this.motorType = DCMotor.getKrakenX44Foc(1);
        break;
      default:
        this.motorType = DCMotor.getKrakenX60(1);
    }
  }

  public TalonFXSimState getSimState() {
    return motor.getSimState();
  }

  public DCMotor getMotorType() {
    return motorType;
  }
}
