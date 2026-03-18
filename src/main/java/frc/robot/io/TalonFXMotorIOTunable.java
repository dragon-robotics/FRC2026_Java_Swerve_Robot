package frc.robot.io;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import frc.robot.util.TalonFXTunableHelper;
import java.util.Optional;

public class TalonFXMotorIOTunable extends TalonFXMotorIO {
  private final TalonFXTunableHelper tunableHelper;

  public TalonFXMotorIOTunable(int canID, TalonFXConfiguration config, String motorName) {
    this(canID, config, motorName, Optional.empty(), Optional.empty());
  }

  // Constructor with Follower config
  public TalonFXMotorIOTunable(
      int canID, TalonFXConfiguration config, String motorName, Follower followerConfig) {
    this(canID, config, motorName, Optional.empty(), Optional.ofNullable(followerConfig));
  }

  // Constructor with CANcoder config
  public TalonFXMotorIOTunable(
      int canID,
      TalonFXConfiguration config,
      String motorName,
      CANcoderConfiguration canCoderConfig) {
    this(canID, config, motorName, Optional.ofNullable(canCoderConfig), Optional.empty());
  }

  // Full Constructor
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
   * Extracts the subsystem name from the motor name. "Intake Roller" → "Intake" "Shooter Hood" →
   * "Shooter" "Climber Winch" → "Climber"
   */
  private static String extractSubsystem(String motorName) {
    int space = motorName.indexOf(' ');
    return (space >= 0) ? motorName.substring(0, space) : motorName;
  }
}
