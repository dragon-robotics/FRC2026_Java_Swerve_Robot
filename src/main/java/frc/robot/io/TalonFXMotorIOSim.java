package frc.robot.io;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Optional;

public class TalonFXMotorIOSim extends TalonFXMotorIO {

    private final DCMotor motorModel;

    public TalonFXMotorIOSim(
            int canID, TalonFXConfiguration config, String motorType, String motorName) {
        this(canID, config, motorType, motorName, Optional.empty(), Optional.empty());
    }

    public TalonFXMotorIOSim(
            int canID,
            TalonFXConfiguration config,
            String motorType,
            String motorName,
            Follower followerConfig) {
        this(canID, config, motorType, motorName, Optional.empty(), Optional.ofNullable(followerConfig));
    }

    public TalonFXMotorIOSim(
            int canID,
            TalonFXConfiguration config,
            String motorType,
            String motorName,
            CANcoderConfiguration canCoderConfig) {
        this(canID, config, motorType, motorName, Optional.ofNullable(canCoderConfig), Optional.empty());
    }

    public TalonFXMotorIOSim(
            int canID,
            TalonFXConfiguration config,
            String motorType,
            String motorName,
            Optional<CANcoderConfiguration> canCoderConfig,
            Optional<Follower> followerConfig) {
        super(canID, config, motorName, canCoderConfig, followerConfig);
        this.motorModel = resolveMotorType(motorType);
    }

    public TalonFXSimState getSimState() {
        return motor.getSimState();
    }

    public DCMotor getMotorType() {
        return motorModel;
    }

    private static DCMotor resolveMotorType(String motorType) {
        return switch (motorType) {
            case "KrakenX60" -> DCMotor.getKrakenX60(1);
            case "KrakenX60_FOC" -> DCMotor.getKrakenX60Foc(1);
            case "KrakenX44" -> DCMotor.getKrakenX44(1);
            case "KrakenX44_FOC" -> DCMotor.getKrakenX44Foc(1);
            default -> DCMotor.getKrakenX60Foc(1);
        };
    }
}