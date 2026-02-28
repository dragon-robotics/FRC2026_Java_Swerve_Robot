package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.util.TalonFXTunableHelper;

public class HopperIOTalonFXTunable extends HopperIOTalonFX {
      private final TalonFXTunableHelper tunableHelper;

    public HopperIOTalonFXTunable(
      int canID, TalonFXConfiguration config, String motorName) {
    super(canID, config, motorName);

    String baseKey = "Hopper/Tuning/TalonFX/" + motorName + "/";
    tunableHelper = new TalonFXTunableHelper(motor, baseKey, config, motorName);
  }

}


