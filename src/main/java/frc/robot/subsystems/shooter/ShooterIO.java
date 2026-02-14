package frc.robot.subsystems.shooter;

import lombok.Getter;
import lombok.Setter;

public interface ShooterIO {

  default void setMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setMotorVoltage not implemented");
  }

  default void setMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setMotorPercentage not implemented");
  }

  default void setMotorRPM(double rpm) {
    throw new UnsupportedOperationException("setMotorRPM not implemented");
  }

  class ShooterIOInputs {
    /** Is the motor connected? */
    @Getter @Setter private boolean motorConnected;

    /** Motor data */
    @Getter @Setter private double motorVoltage;

    @Getter @Setter private double motorDutyCycle;
    @Getter @Setter private double motorCurrent;
    @Getter @Setter private double motorTemperature;
    @Getter @Setter private double motorVelocity;
  }

  default void updateInputs(ShooterIOInputs shooterInputs) {}
}
