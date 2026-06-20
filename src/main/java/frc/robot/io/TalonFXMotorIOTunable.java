package frc.robot.io;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import frc.robot.util.TalonFXTunableHelper;
import java.util.Optional;

/** Talon FX IO wrapper that publishes runtime tuning controls for CTRE configs. */
public class TalonFXMotorIOTunable extends TalonFXMotorIO {
  private final TalonFXTunableHelper tunableHelper;

  /** Creates a tunable Talon FX with no follower or CANcoder. */
  public TalonFXMotorIOTunable(int canID, TalonFXConfiguration config, String motorName) {
    this(canID, config, motorName, Optional.empty(), Optional.empty());
  }

  /** Creates a tunable Talon FX configured as a follower. */
  public TalonFXMotorIOTunable(
      int canID, TalonFXConfiguration config, String motorName, Follower followerConfig) {
    this(canID, config, motorName, Optional.empty(), Optional.ofNullable(followerConfig));
  }

  /** Creates a tunable Talon FX using external CANcoder feedback. */
  public TalonFXMotorIOTunable(
      int canID,
      TalonFXConfiguration config,
      String motorName,
      CANcoderConfiguration canCoderConfig) {
    this(canID, config, motorName, Optional.ofNullable(canCoderConfig), Optional.empty());
  }

  /** Full constructor for selecting optional CANcoder/follower config. */
  public TalonFXMotorIOTunable(
      int canID,
      TalonFXConfiguration config,
      String motorName,
      Optional<CANcoderConfiguration> canCoderConfig,
      Optional<Follower> followerConfig) {
    super(canID, config, motorName, canCoderConfig, followerConfig);

    String subsystem = extractSubsystem(motorName);
    String baseKey = subsystem + "/Tuning/TalonFX/" + motorName + "/";
    tunableHelper = new TalonFXTunableHelper(motor, baseKey, config, motorName);
  }

  /**
   * Extracts the subsystem name from the first word of the motor name.
   *
   * <p>Examples: "Intake Roller" -> "Intake", "Shooter Hood" -> "Shooter".
   */
  private static String extractSubsystem(String motorName) {
    int space = motorName.indexOf(' ');
    return (space >= 0) ? motorName.substring(0, space) : motorName;
  }
}
