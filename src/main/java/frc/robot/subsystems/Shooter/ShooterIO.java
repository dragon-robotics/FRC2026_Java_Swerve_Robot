package frc.robot.subsystems.Shooter;

import lombok.Getter;
import lombok.Setter;

public interface ShooterIO {
    default void runShooter(double rpm) {
        throw new UnsupportedOperationException("runShooter not implemented");
    }
    default void stopShooter() {
        runShooter(0.0);
    }
    default void setDesiredState() {
        throw new UnsupportedOperationException("setDesiredState not implemented");
    }

    default double getShooterSpeed() {
        throw new UnsupportedOperationException("getShooterSpeed not implemented");
    }

    default void getCurrentState() {
        throw new UnsupportedOperationException("getCurrentState not implemented");
    }

    default boolean isShooting() {
        throw new UnsupportedOperationException("isShooting not implemented");
    }
    
    default boolean isShooterStopped() {
        throw new UnsupportedOperationException("is ShooterStoppped is not implemented");
    }

    class ShooterIOInputs {
        /* Shooter motor data */
        @Getter @Setter private double forwardMotorVelocity;
        @Getter @Setter private double inverseMotorVelocity;
        @Getter @Setter private double forwardMotorCurrent; 
        @Getter @Setter private double inverseMotorCurrent;
        @Getter @Setter private boolean isPrep;
    }

    default void updateInputs(ShooterIOInputs shooterInputs) {}
}
