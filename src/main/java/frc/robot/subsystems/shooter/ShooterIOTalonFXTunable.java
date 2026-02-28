package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import frc.robot.util.TalonFXTunableHelper;

public class ShooterIOTalonFXTunable extends ShooterIOTalonFX {

  private final TalonFXTunableHelper tunableHelper;

  public ShooterIOTalonFXTunable(
      int canID, TalonFXConfiguration config, String motorName, Follower followCfg) {
    this(canID, config, motorName); // reuse common init
    motor.setControl(followCfg);
  }

  public ShooterIOTalonFXTunable(int canID, TalonFXConfiguration config, String motorName) {
    super(canID, config, motorName);

    String baseKey = "Shooter/Tuning/TalonFX/" + motorName + "/";
    tunableHelper = new TalonFXTunableHelper(motor, baseKey, config, motorName);
  }
}