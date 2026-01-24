package frc.robot.subsystems.shooter;

import lombok.Getter;
import lombok.Setter;

public interface ShooterIO {
  default void setShooterMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setShooterMotorVoltage is not implemented");
  }

  default void setShooterMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setShooterMotorPercentage is not implemented");
  }

  default void setShooterMotorRPM(double rpm) {
    throw new UnsupportedOperationException("setShooterMotorRPM is not implemented");
  }

  class ShooterIOInputs {

    /** Are the shooter motors connected? */
    @Getter @Setter private boolean shooterMotorConnected;

    /** Shooter motor data */
    @Getter @Setter private double shooterMotorVoltage;

    @Getter @Setter private double shooterMotorDutyCycle;
    @Getter @Setter private double shooterMotorCurrent;
    @Getter @Setter private double shooterMotorTemperature;
    @Getter @Setter private double shooterMotorVelocity;
  }

  default void updateInputs(ShooterIOInputs inputs) {}
}
