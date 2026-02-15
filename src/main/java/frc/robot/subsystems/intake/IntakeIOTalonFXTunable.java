package frc.robot.subsystems.intake;

import java.util.Optional;
import java.util.Set;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.StringSubscriber;

public class IntakeIOTalonFXTunable extends IntakeIOTalonFX {

  private final StringSubscriber controlRequestType;
  private final DoubleSubscriber statorCurrentLimit;
  private final DoubleSubscriber positionSetpoint, velocitySetpoint;
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

  public IntakeIOTalonFXTunable(int canID, TalonFXConfiguration config, String motorName) {
    this(canID, config, motorName, Optional.empty());
  }

  public IntakeIOTalonFXTunable(
      int canID,
      TalonFXConfiguration config,
      String motorName,
      Optional<CANcoderConfiguration> canCoderConfig) {
    super(canID, config, canCoderConfig);

    String baseKey = "Intake/Tuning/TalonFX/" + motorName + "/";

    statorCurrentLimit = DogLog.tunable(baseKey + "StatorCurrentLimit", config.CurrentLimits.StatorCurrentLimit, newLimit -> {
      motor.getConfigurator().apply(config.withCurrentLimits(config.CurrentLimits.withStatorCurrentLimit(newLimit)));
    });

    positionSetpoint = DogLog.tunable(baseKey + "PositionSetpoint", 0.0, newPositionSetpoint -> {
      motor.setControl(motorMotionMagicExpoTorqueCurrentFOCRequest.withPosition(newPositionSetpoint));
    });

    velocitySetpoint = DogLog.tunable(baseKey + "VelocitySetpoint", 0.0, newVelocitySetpoint -> {
      motor.setControl(motorMotionMagicVelocityTorqueCurrentFOCRequest.withVelocity(newVelocitySetpoint / 60));
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

    // If motor name contains "Roller", default to velocity control, otherwise default to position control
    String defaultControlType = motorName.contains("Roller") ? "VelocityTorqueCurrentFOC" : "MotionMagicExpoTorqueCurrentFOC";

    controlRequestType = DogLog.tunable(baseKey + "ControlRequestType", defaultControlType, newControlRequestType -> {
      // No need to apply to motor config, since all control requests are always configured. Just update setpoints based on new control type
      double currentSetpoint = 0.0;
      switch (newControlRequestType) {
        case "PositionVoltage":
          currentSetpoint = positionSetpoint.get();
          motor.setControl(motorPositionVoltageRequest.withPosition(currentSetpoint));
          break;
        case "PositionTorqueCurrentFOC":
          currentSetpoint = positionSetpoint.get();
          motor.setControl(motorPositionTorqueCurrentFOCRequest.withPosition(currentSetpoint));
          break;
        case "MotionMagicVoltage":
          currentSetpoint = positionSetpoint.get();
          motor.setControl(motorMotionMagicVoltageRequest.withPosition(currentSetpoint));
          break;
        case "MotionMagicTorqueCurrentFOC":
          currentSetpoint = positionSetpoint.get();
          motor.setControl(motorMotionMagicTorqueCurrentFOCRequest.withPosition(currentSetpoint));
          break;
        case "MotionMagicExpoVoltage":
          currentSetpoint = positionSetpoint.get();
          motor.setControl(motorMotionMagicExpoVoltageRequest.withPosition(currentSetpoint));
          break;
        case "MotionMagicExpoTorqueCurrentFOC":
          currentSetpoint = positionSetpoint.get();
          motor.setControl(motorMotionMagicExpoTorqueCurrentFOCRequest.withPosition(currentSetpoint));
          break;
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
