package frc.robot.subsystems.climber;

import lombok.Getter;
import lombok.Setter;

public interface ClimberIO {

  default void setElevatorSetpoint(double setPoint) {
    throw new UnsupportedOperationException("setElevatorSetpoint not implemented");
  }

  /* getters */
  default boolean isElevatorStowed(double homeSetPoint, double tolerance) {
    throw new UnsupportedOperationException("isElevatorStowed not implemented");
  }

  default boolean hasClimbed() {
    throw new UnsupportedOperationException("hasClimbed not implemented");
  }

  default boolean getCurrentSpike() {
    throw new UnsupportedOperationException("getCurrentLimit not implemented");
  }

  default double getElevatorPosition() {
    throw new UnsupportedOperationException("getElevatorPosition not implemented");
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

  default void updateInputs(ClimberIOInputs climberInputs) {}
}
