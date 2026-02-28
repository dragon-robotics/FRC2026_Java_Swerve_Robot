package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.util.TalonFXTunableHelper;
import java.util.Optional;

public class IntakeIOTalonFXTunable extends IntakeIOTalonFX {

  private final TalonFXTunableHelper tunableHelper;

  public IntakeIOTalonFXTunable(int canID, TalonFXConfiguration config, String motorName) {
    super(canID, config, motorName);

    String baseKey = "Intake/Tuning/TalonFX/" + motorName + "/";
    tunableHelper = new TalonFXTunableHelper(motor, baseKey, config, motorName);
  }

  // Constructor with CANcoder config
  public IntakeIOTalonFXTunable(
      int canID,
      TalonFXConfiguration config,
      String motorName,
      Optional<com.ctre.phoenix6.configs.CANcoderConfiguration> canCoderConfig) {
    super(canID, config, motorName, canCoderConfig);

    String baseKey = "Intake/Tuning/TalonFX/" + motorName + "/";
    tunableHelper = new TalonFXTunableHelper(motor, baseKey, config, motorName);
  }
}
