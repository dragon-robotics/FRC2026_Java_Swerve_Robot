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
        /** are shooter motors connected */
        @Getter @Setter private boolean forwardMotorConnected;
        @Getter @Setter private boolean inverseMotorConnected;
        // shooterSpeed
        @Getter @Setter private double forwardMotorVelocity;
        @Getter @Setter private double inverseMotorVelocity;

        // forward motor data
        @Getter @Setter private double forwardMotorCurrent; 
        @Getter @Setter private double forwardMotorVoltage;
        @Getter @Setter private double forwardMotorTemprature; 
        // inverse motor data
        @Getter @Setter private double inverseMotorVoltage;
        @Getter @Setter private double inverseMotorCurrent;
        @Getter @Setter private double inverseMotorTemprature;

    }
    
    default void updateInputs(ShooterIOInputs shooterInputs) {}
}
