package frc.robot.io;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import java.util.Optional;

public class TalonFXMotorIO implements MotorIO {

    protected final TalonFX motor;
    protected final int canID;
    protected final TalonFXConfiguration config;
    protected final String motorName;
    protected CANcoder canCoder;

    protected final VelocityTorqueCurrentFOC velocityRequest;
    protected final MotionMagicVelocityTorqueCurrentFOC mmVelocityRequest;
    protected final MotionMagicExpoTorqueCurrentFOC mmPositionRequest;

    /** Standard motor (no follower, no CANcoder). */
    public TalonFXMotorIO(int canID, TalonFXConfiguration config, String motorName) {
        this(canID, config, motorName, Optional.empty(), Optional.empty());
    }

    /** Follower motor. */
    public TalonFXMotorIO(
            int canID, TalonFXConfiguration config, String motorName, Follower followerConfig) {
        this(canID, config, motorName, Optional.empty(), Optional.of(followerConfig));
    }

    /** Motor with external CANcoder feedback. */
    public TalonFXMotorIO(
            int canID,
            TalonFXConfiguration config,
            String motorName,
            CANcoderConfiguration canCoderConfig) {
        this(canID, config, motorName, Optional.of(canCoderConfig), Optional.empty());
    }

    /** Full constructor. */
    public TalonFXMotorIO(
            int canID,
            TalonFXConfiguration config,
            String motorName,
            Optional<CANcoderConfiguration> canCoderConfig,
            Optional<Follower> followerConfig) {
        this.canID = canID;
        this.config = config;
        this.motorName = motorName;

        motor = new TalonFX(canID);
        motor.clearStickyFaults();

        velocityRequest = new VelocityTorqueCurrentFOC(0);
        mmVelocityRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
        mmPositionRequest = new MotionMagicExpoTorqueCurrentFOC(0);

        // Apply CANcoder config if present
        canCoderConfig.ifPresent(
                ccCfg -> {
                    canCoder = new CANcoder(canID);
                    canCoder.getConfigurator().apply(ccCfg);
                    TalonFXConfiguration updatedConfig = config.withFeedback(
                            new FeedbackConfigs()
                                    .withFeedbackRemoteSensorID(canCoder.getDeviceID())
                                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));
                    motor.getConfigurator().apply(updatedConfig);
                });

        if (canCoderConfig.isEmpty()) {
            motor.getConfigurator().apply(config);
        }

        // Apply follower config if present
        followerConfig.ifPresent(motor::setControl);
    }

    @Override
    public String getMotorName() {
        return motorName;
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
        motor.setControl(velocityRequest.withVelocity(rpm / 60.0));
    }

    @Override
    public void setMotorPosition(double setpoint) {
        motor.setControl(mmPositionRequest.withPosition(setpoint));
    }

    @Override
    public void setMotorPosition(double setpoint, int slotID) {
        motor.setControl(mmPositionRequest.withPosition(setpoint).withSlot(slotID));
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.setMotorConnected(motor.isConnected());
        inputs.setMotorVoltage(motor.getMotorVoltage().getValueAsDouble());
        inputs.setMotorCurrent(motor.getStatorCurrent().getValueAsDouble());
        inputs.setMotorTemperature(motor.getDeviceTemp().getValueAsDouble());
        inputs.setMotorVelocity(motor.getVelocity().getValueAsDouble());
        inputs.setMotorPosition(motor.getPosition().getValueAsDouble());
    }

    /** Expose for sim subclasses. */
    public TalonFX getMotor() {
        return motor;
    }
}