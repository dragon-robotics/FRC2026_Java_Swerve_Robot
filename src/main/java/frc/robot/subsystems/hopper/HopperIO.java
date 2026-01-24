package frc.robot.subsystems.hopper;

import lombok.Getter;
import lombok.Setter;

public interface HopperIO {
  default void setHopperRollerMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setHopperRollerMotorVoltage is not implemented");
  }

  default void setHopperRollerMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setHopperRollerMotorPercentage is not implemented");
  }

  default void setHopperRollerMotorRPM(double rpm) {
    throw new UnsupportedOperationException("setHopperRollerMotorRPM is not implemented");
  }

  default void setHopperExtensionMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setHopperExtensionMotorVoltage is not implemented");
  }

  default void setHopperExtensionMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setHopperExtensionMotorPercentage is not implemented");
  }

  default void setHopperExtensionMotorSetpoint(double setpoint) {
    throw new UnsupportedOperationException("setHopperExtensionMotorSetpoint is not implemented");
  }

  class HopperIOInputs {

    /** Are the hopper motors connected? */
    @Getter @Setter private boolean hopperRollerMotorConnected;

    @Getter @Setter private boolean hopperExtensionMotorConnected;

    /** Hopper roller motor data */
    @Getter @Setter private double hopperRollerMotorVoltage;

    @Getter @Setter private double hopperRollerMotorDutyCycle;
    @Getter @Setter private double hopperRollerMotorCurrent;
    @Getter @Setter private double hopperRollerMotorTemperature;
    @Getter @Setter private double hopperRollerMotorVelocity;

    /** Hopper extension motor data */
    @Getter @Setter private double hopperExtensionMotorVoltage;

    @Getter @Setter private double hopperExtensionMotorDutyCycle;
    @Getter @Setter private double hopperExtensionMotorCurrent;
    @Getter @Setter private double hopperExtensionMotorTemperature;
    @Getter @Setter private double hopperExtensionMotorSetpoint;
  }

  default void updateInputs(HopperIOInputs inputs) {}
}
