package frc.robot.subsystems.shooter;

import java.util.Set;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.StringSubscriber;

public class ShooterIOTalonFXTunable extends ShooterIOTalonFX {

  private final StringSubscriber controlRequestType;
  private final DoubleSubscriber statorCurrentlimit;
  private final DoubleSubscriber velocitySetpoint;
  private final DoubleSubscriber kP, kI, kD, kS, kV, kA;
  private final DoubleSubscriber expo_kV, expo_kA;

  private static final Set<String> validControlRequestTypes =
      Set.of(
        "PositionVoltage",
        "PositionTorqueCurrentFOC",
        "MotionMagicVoltage",
        "MotionMagicTorqueCurrentFOC",
        "MotionMagicExpoVoltage",
        "MotionMagicExpoTorqueCurrentFOC",
        "MotionMagicVelocityVoltage",
        "MotionMagicVelocityTorqueCurrentFOC",
        "VelocityVoltage",
        "VelocityTorqueCurrentFOC"
      );

  public ShooterIOTalonFXTunable(int canID, TalonFXConfiguration config, String motorName) {
    super(canID, config);

    String baseKey = "Shooter/Tuning/TalonFX/" + motorName + "/";

    statorCurrentlimit = DogLog.tunable(baseKey + "StatorCurrentLimit", config.CurrentLimits.StatorCurrentLimit, newLimit -> {
      motor.getConfigurator().apply(config.withCurrentLimits(config.CurrentLimits.withStatorCurrentLimit(newLimit)));
    });

    velocitySetpoint = DogLog.tunable(baseKey + "VelocitySetpoint", 0.0, newVelocitySetpoint -> {
      motor.setControl(motorVelocityTorqueCurrentFOCRequest.withVelocity(newVelocitySetpoint / 60));
    });

    kP = DogLog.tunable(baseKey + "kP", config.Slot0.kP, newP -> {
      motor.getConfigurator().apply(config.withSlot0(config.Slot0.withKP(newP)));
    });
    kI = DogLog.tunable(baseKey + "kI", config.Slot0.kI, newI -> {
      motor.getConfigurator().apply(config.withSlot0(config.Slot0.withKI(newI)));
    });
    kD = DogLog.tunable(baseKey + "kD", config.Slot0.kD, newD -> {
      motor.getConfigurator().apply(config.withSlot0(config.Slot0.withKD(newD)));
    });
    kS = DogLog.tunable(baseKey + "kS", config.Slot0.kS, newS -> {
      motor.getConfigurator().apply(config.withSlot0(config.Slot0.withKS(newS)));
    });
    kV = DogLog.tunable(baseKey + "kV", config.Slot0.kV, newV -> {
      motor.getConfigurator().apply(config.withSlot0(config.Slot0.withKV(newV)));
    });
    kA = DogLog.tunable(baseKey + "kA", config.Slot0.kA, newA -> {
      motor.getConfigurator().apply(config.withSlot0(config.Slot0.withKA(newA)));
    });

    expo_kV = DogLog.tunable(baseKey + "expo_kV", config.MotionMagic.MotionMagicExpo_kV, newExpoKV -> {
      motor.getConfigurator().apply(config.withMotionMagic(config.MotionMagic.withMotionMagicExpo_kV(newExpoKV)));
    });
    expo_kA = DogLog.tunable(baseKey + "expo_kA", config.MotionMagic.MotionMagicExpo_kA, newExpoKA -> {
      motor.getConfigurator().apply(config.withMotionMagic(config.MotionMagic.withMotionMagicExpo_kA(newExpoKA)));
    });

    controlRequestType = DogLog.tunable(baseKey + "ControlRequestType", "VelocityVoltage", newControlRequestType -> {
      // No need to apply to motor config, since all control requests are always configured. Just update setpoints based on new control type
      double currentSetpoint = 0.0;
      switch (newControlRequestType) {
        case "MotionMagicVelocityVoltage":
          currentSetpoint = velocitySetpoint.get();
          motor.setControl(motorMotionMagicVelocityVoltageRequest.withVelocity(currentSetpoint / 60));
          break;
        case "MotionMagicVelocityTorqueCurrentFOC":
          currentSetpoint = velocitySetpoint.get();
          motor.setControl(motorMotionMagicVelocityTorqueCurrentFOCRequest.withVelocity(currentSetpoint / 60));
          break;
        case "VelocityVoltage":
          currentSetpoint = velocitySetpoint.get();
          motor.setControl(motorVelocityVoltageRequest.withVelocity(currentSetpoint / 60));
          break;
        case "VelocityTorqueCurrentFOC":
           currentSetpoint = velocitySetpoint.get();
           motor.setControl(motorVelocityTorqueCurrentFOCRequest.withVelocity(currentSetpoint / 60));
           break;
      }
    });    
  } 
}
