package frc.robot.io;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Optional;

/** Simulation-backed Talon FX IO wrapper. */
public class TalonFXMotorIOSim extends TalonFXMotorIO {
  private static final String KRAKEN_X60 = "KrakenX60";
  private static final String KRAKEN_X60_FOC = "KrakenX60_FOC";
  private static final String KRAKEN_X44 = "KrakenX44";
  private static final String KRAKEN_X44_FOC = "KrakenX44_FOC";

  private final DCMotor motorModel;

  /** Creates a simulated Talon FX with no follower or CANcoder. */
  public TalonFXMotorIOSim(
      int canID, TalonFXConfiguration config, String motorType, String motorName) {
    this(canID, config, motorType, motorName, Optional.empty(), Optional.empty());
  }

  /** Creates a simulated Talon FX configured as a follower. */
  public TalonFXMotorIOSim(
      int canID,
      TalonFXConfiguration config,
      String motorType,
      String motorName,
      Follower followerConfig) {
    this(
        canID, config, motorType, motorName, Optional.empty(), Optional.ofNullable(followerConfig));
  }

  /** Creates a simulated Talon FX using external CANcoder feedback. */
  public TalonFXMotorIOSim(
      int canID,
      TalonFXConfiguration config,
      String motorType,
      String motorName,
      CANcoderConfiguration canCoderConfig) {
    this(
        canID, config, motorType, motorName, Optional.ofNullable(canCoderConfig), Optional.empty());
  }

  /** Full constructor for selecting optional CANcoder/follower config and motor model. */
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

  /** Returns the Phoenix simulation state for this Talon FX. */
  public TalonFXSimState getSimState() {
    return motor.getSimState();
  }

  /** Returns the WPILib motor model used by mechanism simulations. */
  public DCMotor getMotorType() {
    return motorModel;
  }

  private static DCMotor resolveMotorType(String motorType) {
    return switch (motorType) {
      case KRAKEN_X60 -> DCMotor.getKrakenX60(1);
      case KRAKEN_X60_FOC -> DCMotor.getKrakenX60Foc(1);
      case KRAKEN_X44 -> DCMotor.getKrakenX44(1);
      case KRAKEN_X44_FOC -> DCMotor.getKrakenX44Foc(1);
      default -> DCMotor.getKrakenX60Foc(1);
    };
  }
}
