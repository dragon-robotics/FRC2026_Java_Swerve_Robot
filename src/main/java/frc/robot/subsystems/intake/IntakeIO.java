package frc.robot.subsystems.intake;

import lombok.Getter;
import lombok.Setter;

public interface IntakeIO {
  default void setIntakeRollerMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setIntakeRollerMotorVoltage is not implemented");
  }

  default void setIntakeRollerMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setIntakeRollerMotorPercentage is not implemented");
  }

  default void setIntakeRollerMotorRPM(double rpm) {
    throw new UnsupportedOperationException("setIntakeRollerMotorRPM is not implemented");
  }

  default void setIntakeArmMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setIntakeArmMotorVoltage is not implemented");
  }

  default void setIntakeArmMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setIntakeArmMotorPercentage is not implemented");
  }

  default void setIntakeArmMotorSetpoint(double setpoint) {
    throw new UnsupportedOperationException("setIntakeArmMotorSetpoint is not implemented");
  }

  class IntakeIOInputs {

    /** Are the intake motors connected? */
    @Getter @Setter private boolean intakeRollerMotorConnected;

    @Getter @Setter private boolean intakeArmMotorConnected;

    /** Intake roller motor data */
    @Getter @Setter private double intakeRollerMotorVoltage;
    @Getter @Setter private double intakeRollerMotorDutyCycle;
    @Getter @Setter private double intakeRollerMotorCurrent;
    @Getter @Setter private double intakeRollerMotorTemperature;
    @Getter @Setter private double intakeRollerMotorVelocity;

    /** Intake arm motor data */
    @Getter @Setter private double intakeArmMotorVoltage;
    @Getter @Setter private double intakeArmMotorDutyCycle;
    @Getter @Setter private double intakeArmMotorCurrent;
    @Getter @Setter private double intakeArmMotorTemperature;
    @Getter @Setter private double intakeArmMotorSetpoint;
  }

  default void updateInputs(IntakeIOInputs inputs) {}
}
