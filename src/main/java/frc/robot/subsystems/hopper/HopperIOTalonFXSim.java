package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;

public class HopperIOTalonFXSim extends HopperIOTalonFX {

    private final DCMotor motorType;

    public HopperIOTalonFXSim(int canID, TalonFXConfiguration config, String motorType) {

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

    public TalonFXSimState getSimState() {
        return motor.getSimState();
    }

    public DCMotor getMotorType() {
        return motorType;
    }
}
