package frc.robot.subsystems.climber;

import lombok.Getter;
import lombok.Setter;

public interface ClimberIO {

  default void setMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setMotorVoltage not implemented");
  }

  default void setMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setMotorPercentage not implemented");
  }

  default void setMotorRPM(double rpm) {
    throw new UnsupportedOperationException("setMotorRPM not implemented");
  }

  default void setMotorPosition(double setpoint) {
    throw new UnsupportedOperationException("setMotorPosition is not implemented");
  }

  class ClimberIOInputs {
    // Motor connection status
    @Getter @Setter private boolean motorConnected;

    // Lead motor data
    @Getter @Setter private double motorPosition;
    @Getter @Setter private double motorVelocity;
    @Getter @Setter private double motorVoltage;
    @Getter @Setter private double motorCurrent;
    @Getter @Setter private double motorTemperature;
  }

  default void updateInputs(ClimberIOInputs climberInputs) {
  }
}
