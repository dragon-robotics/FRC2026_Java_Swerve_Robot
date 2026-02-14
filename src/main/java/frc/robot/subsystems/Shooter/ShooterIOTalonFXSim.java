package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;

public class ShooterIOTalonFXSim extends ShooterIOTalonFX{
    
  private final DCMotor motorType;
    
  public ShooterIOTalonFXSim(
      int canID,
      TalonFXConfiguration config,
      String motorType) {
    
    super(canID, config);

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

  public ShooterIOTalonFXSim(
      int canID,
      TalonFXConfiguration config,
      int leadCANID,
      String motorType) {
    
    super(canID, config, leadCANID);

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
