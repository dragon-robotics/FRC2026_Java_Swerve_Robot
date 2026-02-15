package frc.robot.subsystems.climber;

import java.util.Optional;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class ClimberIOSparkMax implements ClimberIO {
    protected final SparkMax motor;
    protected final int canID;
    protected final SparkMaxConfig config;
    protected final String motorType;

    private final SparkClosedLoopController motorController;
    private final Optional<AbsoluteEncoderConfig> absEncoderConfig;

    public ClimberIOSparkMax(int canID, SparkMaxConfig config, String motorType) {
        this(canID, config, motorType, Optional.empty());
    }

    public ClimberIOSparkMax(
            int canID,
            SparkMaxConfig config,
            String motorType,
            Optional<AbsoluteEncoderConfig> absEncoderConfig) {
        this.canID = canID;
        this.config = config;
        this.motorType = motorType;
        this.absEncoderConfig = absEncoderConfig;

        // Instantiate the motors and encoders//
        motor = new SparkMax(this.canID, MotorType.kBrushless);
        motorController = motor.getClosedLoopController();

        // Clear any existing faults//
        motor.clearFaults();

        this.absEncoderConfig.ifPresentOrElse(
                absEncCfg -> {
                    this.config.apply(absEncCfg);
                    motor.configure(
                            this.config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                    motor.getEncoder().setPosition(0);
                }, () -> {
                    motor.configure(
                            this.config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                    motor.getEncoder().setPosition(0);
                });
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setMotorPercentage(double percentage) {
        motor.set(percentage);
    }

    @Override
    public void setMotorRPM(double rpm) {
        motorController.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void setMotorPosition(double position) {
        motorController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void updateInputs(ClimberIOInputs climberInputs) {
        // Motor connection status
        climberInputs.setLeadMotorConnected(motor.getDeviceId() == canID);

        // Lead motor data
        climberInputs.setLeadMotorPosition(motor.getEncoder().getPosition());
        climberInputs.setLeadMotorVelocity(motor.getEncoder().getVelocity());
        climberInputs.setLeadMotorVoltage(motor.getAppliedOutput() * motor.getBusVoltage());
        climberInputs.setLeadMotorCurrent(motor.getOutputCurrent());
        climberInputs.setLeadMotorTemperature(motor.getMotorTemperature());
    }

}