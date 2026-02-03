package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeSubsystemConstants.*;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX intakeRollerMotor;
  private final TalonFX intakeArmMotor;

  private final MotionMagicVelocityTorqueCurrentFOC
      rollerMotionMagicVelocityTorqueCurrentFOCRequest;
  private final MotionMagicExpoTorqueCurrentFOC intakeArmMotionMagicExpoTorqueCurrentFOCRequest;

  public IntakeIOTalonFX() {

    /* Instantiate the motors */
    intakeRollerMotor = new TalonFX(INTAKE_ROLLER_MOTOR_ID);
    intakeArmMotor = new TalonFX(INTAKE_ARM_MOTOR_ID);

    /* Clear any existing faults */
    intakeRollerMotor.clearStickyFaults();
    intakeArmMotor.clearStickyFaults();

    /* Configure the motors */
    /* Roller motor configuration */
    TalonFXConfiguration rollerConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(INTAKE_ROLLER_STATOR_CURRENT_LIMIT))
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(INTAKE_ROLLER_SUPPLY_CURRENT_LIMIT)))
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(Volts.of(INTAKE_ROLLER_MAX_VOLTAGE))
                    .withPeakReverseVoltage(Volts.of(-INTAKE_ROLLER_MAX_VOLTAGE)))
            .withOpenLoopRamps(
                new OpenLoopRampsConfigs()
                    .withDutyCycleOpenLoopRampPeriod(Seconds.of(INTAKE_ROLLER_RAMP_RATE))
                    .withTorqueOpenLoopRampPeriod(Seconds.of(INTAKE_ROLLER_RAMP_RATE))
                    .withVoltageOpenLoopRampPeriod(Seconds.of(INTAKE_ROLLER_RAMP_RATE)))
            .withClosedLoopRamps(
                new ClosedLoopRampsConfigs()
                    .withDutyCycleClosedLoopRampPeriod(Seconds.of(INTAKE_ROLLER_RAMP_RATE))
                    .withTorqueClosedLoopRampPeriod(Seconds.of(INTAKE_ROLLER_RAMP_RATE))
                    .withVoltageClosedLoopRampPeriod(Seconds.of(INTAKE_ROLLER_RAMP_RATE)))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(INTAKE_ROLLER_NEUTRAL_MODE));

    /* Roller motor PID configs for Velocity Control */
    Slot0Configs rollerSlot0Configs = rollerConfig.Slot0; // @TODO: NEEDS TO BE TESTED AND TUNED
    rollerSlot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    rollerSlot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    rollerSlot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    rollerSlot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    rollerSlot0Configs.kI = 0; // no output for integrated error
    rollerSlot0Configs.kD = 0; // no output for error derivative

    // set Motion Magic Velocity settings
    MotionMagicConfigs rollerMotionMagicConfigs = rollerConfig.MotionMagic;
    rollerMotionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    rollerMotionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    /* Arm motor configuration */
    TalonFXConfiguration armConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(INTAKE_ARM_STATOR_CURRENT_LIMIT))
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(INTAKE_ARM_SUPPLY_CURRENT_LIMIT)))
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(Volts.of(INTAKE_ARM_MAX_VOLTAGE))
                    .withPeakReverseVoltage(Volts.of(-INTAKE_ARM_MAX_VOLTAGE)))
            .withOpenLoopRamps(
                new OpenLoopRampsConfigs()
                    .withDutyCycleOpenLoopRampPeriod(Seconds.of(INTAKE_ARM_RAMP_RATE))
                    .withTorqueOpenLoopRampPeriod(Seconds.of(INTAKE_ARM_RAMP_RATE))
                    .withVoltageOpenLoopRampPeriod(Seconds.of(INTAKE_ARM_RAMP_RATE)))
            .withClosedLoopRamps(
                new ClosedLoopRampsConfigs()
                    .withDutyCycleClosedLoopRampPeriod(Seconds.of(INTAKE_ARM_RAMP_RATE))
                    .withTorqueClosedLoopRampPeriod(Seconds.of(INTAKE_ARM_RAMP_RATE))
                    .withVoltageClosedLoopRampPeriod(Seconds.of(INTAKE_ARM_RAMP_RATE)))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(INTAKE_ARM_NEUTRAL_MODE));

    /* Arm motor PID configs for Position Control */
    Slot0Configs armSlot0Configs = armConfig.Slot0; // @TODO: NEEDS TO BE TESTED AND TUNED
    armSlot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    armSlot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    armSlot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    armSlot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    armSlot0Configs.kI = 0; // no output for integrated error
    armSlot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic Expo settings
    var armMotionMagicConfigs = armConfig.MotionMagic;
    armMotionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
    armMotionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
    armMotionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    intakeRollerMotor.getConfigurator().apply(rollerConfig);
    intakeArmMotor.getConfigurator().apply(armConfig);

    /* Create Motion Magic Velocity and Motion Magic Expo requests */
    rollerMotionMagicVelocityTorqueCurrentFOCRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
    intakeArmMotionMagicExpoTorqueCurrentFOCRequest = new MotionMagicExpoTorqueCurrentFOC(0);
  }

  @Override
  public void setIntakeRollerMotorVoltage(double voltage) {
    intakeRollerMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeRollerMotorPercentage(double percentage) {
    intakeRollerMotor.set(percentage);
  }

  @Override
  public void setIntakeRollerMotorRPM(double rpm) {
    intakeRollerMotor.setControl(
        rollerMotionMagicVelocityTorqueCurrentFOCRequest.withVelocity(rpm / 60));
  }

  @Override
  public void setIntakeArmMotorVoltage(double voltage) {
    intakeArmMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeArmMotorPercentage(double percentage) {
    intakeArmMotor.set(percentage);
  }

  @Override
  public void setIntakeArmMotorSetpoint(double setpoint) {
    intakeArmMotor.setControl(
        intakeArmMotionMagicExpoTorqueCurrentFOCRequest.withPosition(setpoint));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    inputs.setIntakeRollerMotorConnected(intakeRollerMotor.isConnected());
    inputs.setIntakeArmMotorConnected(intakeArmMotor.isConnected());

    inputs.setIntakeRollerMotorVoltage(intakeRollerMotor.getMotorVoltage().getValueAsDouble());
    inputs.setIntakeRollerMotorDutyCycle(intakeRollerMotor.getDutyCycle().getValueAsDouble());
    inputs.setIntakeRollerMotorCurrent(intakeRollerMotor.getStatorCurrent().getValueAsDouble());
    inputs.setIntakeRollerMotorTemperature(intakeRollerMotor.getDeviceTemp().getValueAsDouble());
    inputs.setIntakeRollerMotorVelocity(intakeRollerMotor.getVelocity().getValueAsDouble());

    inputs.setIntakeArmMotorVoltage(intakeArmMotor.getMotorVoltage().getValueAsDouble());
    inputs.setIntakeArmMotorDutyCycle(intakeArmMotor.getDutyCycle().getValueAsDouble());
    inputs.setIntakeArmMotorCurrent(intakeArmMotor.getStatorCurrent().getValueAsDouble());
    inputs.setIntakeArmMotorTemperature(intakeArmMotor.getDeviceTemp().getValueAsDouble());
    inputs.setIntakeArmMotorSetpoint(intakeArmMotor.getPosition().getValueAsDouble());
  }
}
