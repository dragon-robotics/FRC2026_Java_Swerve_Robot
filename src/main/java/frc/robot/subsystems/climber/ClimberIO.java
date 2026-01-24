package frc.robot.subsystems.climber;

import lombok.Getter;
import lombok.Setter;

public interface ClimberIO {

    default void setClimberMotorVoltage(double voltage) {
        throw new UnsupportedOperationException("setClimberMotorVoltage is not implemented");
    }

    default void setClimberMotorPercentage(double percentage) {
        throw new UnsupportedOperationException("setClimberMotorPercentage is not implemented");
    }

    default void setClimberSetpoint(double setpoint) {
        throw new UnsupportedOperationException("setClimberSetpoint is not implemented");
    }

    class ClimberIOInputs {

        /** Are the climber motors connected? */
        @Getter @Setter private boolean climberMotorConnected;

        /** Climber motor data */
        @Getter @Setter private double climberMotorVoltage;
        @Getter @Setter private double climberMotorDutyCycle;
        @Getter @Setter private double climberMotorCurrent;
        @Getter @Setter private double climberMotorTemperature;
        @Getter @Setter private double climberMotorSetpoint;
        @Getter @Setter private double climberMotorVelocity;
    }

    default void updateInputs(ClimberIOInputs inputs) {}
}
