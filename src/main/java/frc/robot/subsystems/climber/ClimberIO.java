package frc.robot.subsystems.climber;

import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
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
    @Getter @Setter private boolean leadMotorConnected;
    @Getter @Setter private boolean followerMotorConnected;

    // Lead motor data
    @Getter @Setter private double leadMotorPosition;
    @Getter @Setter private double leadMotorVelocity;
    @Getter @Setter private double leadMotorVoltage;
    @Getter @Setter private double leadMotorCurrent;
    @Getter @Setter private double leadMotorTemperature;

    // Follower motor data
    @Getter @Setter private double followerMotorPosition;
    @Getter @Setter private double followerMotorVelocity;
    @Getter @Setter private double followerMotorVoltage;
    @Getter @Setter private double followerMotorCurrent;
    @Getter @Setter private double followerMotorTemperature;
  }

  default void updateInputs(ClimberIOInputs climberInputs) {
  }
}
