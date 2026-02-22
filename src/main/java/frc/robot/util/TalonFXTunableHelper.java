package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.StringSubscriber;

public class TalonFXTunableHelper {

  // Position Controls //
  protected final PositionVoltage motorPositionVoltageRequest;
  protected final PositionTorqueCurrentFOC motorPositionTorqueCurrentFOCRequest;
  protected final MotionMagicVoltage motorMotionMagicVoltageRequest;
  protected final MotionMagicTorqueCurrentFOC motorMotionMagicTorqueCurrentFOCRequest;
  protected final MotionMagicExpoVoltage motorMotionMagicExpoVoltageRequest;
  protected final MotionMagicExpoTorqueCurrentFOC motorMotionMagicExpoTorqueCurrentFOCRequest;

  // Velocity Controls //
  protected final MotionMagicVelocityVoltage motorMotionMagicVelocityVoltageRequest;
  protected final MotionMagicVelocityTorqueCurrentFOC
      motorMotionMagicVelocityTorqueCurrentFOCRequest;
  protected final VelocityVoltage motorVelocityVoltageRequest;
  protected final VelocityTorqueCurrentFOC motorVelocityTorqueCurrentFOCRequest;

  private final StringSubscriber controlRequestType;
  private final DoubleSubscriber statorCurrentLimit;
  private final DoubleSubscriber targetPositionSetpoint, targetVelocitySetpoint;
  private final DoubleSubscriber kP, kI, kD, kS, kV, kA;
  private final DoubleSubscriber expo_kV, expo_kA;

  public static final String[] CONTROL_STRATEGIES =
      new String[] {
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
      };

  public TalonFXTunableHelper(
      TalonFX motor, String baseKey, TalonFXConfiguration config, String motorName) {
    // Position Control Requests
    motorPositionVoltageRequest = new PositionVoltage(0);
    motorPositionTorqueCurrentFOCRequest = new PositionTorqueCurrentFOC(0);
    motorMotionMagicVoltageRequest = new MotionMagicVoltage(0);
    motorMotionMagicTorqueCurrentFOCRequest = new MotionMagicTorqueCurrentFOC(0);
    motorMotionMagicExpoVoltageRequest = new MotionMagicExpoVoltage(0);
    motorMotionMagicExpoTorqueCurrentFOCRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    // Velocity Control Requests
    motorVelocityVoltageRequest = new VelocityVoltage(0);
    motorVelocityTorqueCurrentFOCRequest = new VelocityTorqueCurrentFOC(0);
    motorMotionMagicVelocityVoltageRequest = new MotionMagicVelocityVoltage(0);
    motorMotionMagicVelocityTorqueCurrentFOCRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

    statorCurrentLimit =
        DogLog.tunable(
            baseKey + "StatorCurrentLimit",
            config.CurrentLimits.StatorCurrentLimit,
            newLimit -> {
              motor
                  .getConfigurator()
                  .apply(
                      config.withCurrentLimits(
                          config.CurrentLimits.withStatorCurrentLimit(newLimit)));
            });

    kP =
        DogLog.tunable(
            baseKey + "kP",
            config.Slot0.kP,
            newP -> {
              motor.getConfigurator().apply(config.withSlot0(config.Slot0.withKP(newP)));
            });
    kI =
        DogLog.tunable(
            baseKey + "kI",
            config.Slot0.kI,
            newI -> {
              motor.getConfigurator().apply(config.withSlot0(config.Slot0.withKI(newI)));
            });
    kD =
        DogLog.tunable(
            baseKey + "kD",
            config.Slot0.kD,
            newD -> {
              motor.getConfigurator().apply(config.withSlot0(config.Slot0.withKD(newD)));
            });
    kS =
        DogLog.tunable(
            baseKey + "kS",
            config.Slot0.kS,
            newS -> {
              motor.getConfigurator().apply(config.withSlot0(config.Slot0.withKS(newS)));
            });
    kV =
        DogLog.tunable(
            baseKey + "kV",
            config.Slot0.kV,
            newV -> {
              motor.getConfigurator().apply(config.withSlot0(config.Slot0.withKV(newV)));
            });
    kA =
        DogLog.tunable(
            baseKey + "kA",
            config.Slot0.kA,
            newA -> {
              motor.getConfigurator().apply(config.withSlot0(config.Slot0.withKA(newA)));
            });

    expo_kV =
        DogLog.tunable(
            baseKey + "expo_kV",
            config.MotionMagic.MotionMagicExpo_kV,
            newExpoKV -> {
              motor
                  .getConfigurator()
                  .apply(
                      config.withMotionMagic(config.MotionMagic.withMotionMagicExpo_kV(newExpoKV)));
            });
    expo_kA =
        DogLog.tunable(
            baseKey + "expo_kA",
            config.MotionMagic.MotionMagicExpo_kA,
            newExpoKA -> {
              motor
                  .getConfigurator()
                  .apply(
                      config.withMotionMagic(config.MotionMagic.withMotionMagicExpo_kA(newExpoKA)));
            });

    targetPositionSetpoint =
        DogLog.tunable(
            baseKey + "PositionSetpoint",
            0.0,
            newPositionSetpoint -> {
              motor.setControl(
                  motorMotionMagicExpoTorqueCurrentFOCRequest.withPosition(newPositionSetpoint));
            });

    targetVelocitySetpoint =
        DogLog.tunable(
            baseKey + "VelocitySetpoint",
            0.0,
            newVelocitySetpoint -> {
              motor.setControl(
                  motorMotionMagicVelocityTorqueCurrentFOCRequest.withVelocity(
                      newVelocitySetpoint / 60));
            });

    // If motor name contains "Roller", default to velocity control, otherwise
    // default to position control
    String defaultControlType =
        motorName.contains("Roller")
            ? "VelocityTorqueCurrentFOC"
            : "MotionMagicExpoTorqueCurrentFOC";

    controlRequestType =
        DogLog.tunable(
            baseKey + "ControlRequestType",
            defaultControlType,
            newControlRequestType -> {
              // No need to apply to motor config, since all control requests are always
              // configured. Just update setpoints based on new control type
              double currentSetpoint = 0.0;
              switch (newControlRequestType) {
                case "PositionVoltage":
                  currentSetpoint = targetPositionSetpoint.get();
                  motor.setControl(motorPositionVoltageRequest.withPosition(currentSetpoint));
                  break;
                case "PositionTorqueCurrentFOC":
                  currentSetpoint = targetPositionSetpoint.get();
                  motor.setControl(
                      motorPositionTorqueCurrentFOCRequest.withPosition(currentSetpoint));
                  break;
                case "MotionMagicVoltage":
                  currentSetpoint = targetPositionSetpoint.get();
                  motor.setControl(motorMotionMagicVoltageRequest.withPosition(currentSetpoint));
                  break;
                case "MotionMagicTorqueCurrentFOC":
                  currentSetpoint = targetPositionSetpoint.get();
                  motor.setControl(
                      motorMotionMagicTorqueCurrentFOCRequest.withPosition(currentSetpoint));
                  break;
                case "MotionMagicExpoVoltage":
                  currentSetpoint = targetPositionSetpoint.get();
                  motor.setControl(
                      motorMotionMagicExpoVoltageRequest.withPosition(currentSetpoint));
                  break;
                case "MotionMagicExpoTorqueCurrentFOC":
                  currentSetpoint = targetPositionSetpoint.get();
                  motor.setControl(
                      motorMotionMagicExpoTorqueCurrentFOCRequest.withPosition(currentSetpoint));
                  break;
                case "MotionMagicVelocityVoltage":
                  currentSetpoint = targetVelocitySetpoint.get();
                  motor.setControl(
                      motorMotionMagicVelocityVoltageRequest.withVelocity(currentSetpoint / 60));
                  break;
                case "MotionMagicVelocityTorqueCurrentFOC":
                  currentSetpoint = targetVelocitySetpoint.get();
                  motor.setControl(
                      motorMotionMagicVelocityTorqueCurrentFOCRequest.withVelocity(
                          currentSetpoint / 60));
                  break;
                case "VelocityVoltage":
                  currentSetpoint = targetVelocitySetpoint.get();
                  motor.setControl(motorVelocityVoltageRequest.withVelocity(currentSetpoint / 60));
                  break;
                case "VelocityTorqueCurrentFOC":
                  currentSetpoint = targetVelocitySetpoint.get();
                  motor.setControl(
                      motorVelocityTorqueCurrentFOCRequest.withVelocity(currentSetpoint / 60));
                  break;
              }
            });
  }
}
