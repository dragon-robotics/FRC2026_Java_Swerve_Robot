package frc.robot.subsystems.hopper;

// import frc.robot.subsystems.hopper.HopperIO.HopperIOInputs;
import lombok.Getter;
import lombok.Setter;

public interface HopperIO {

  default void setMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setMotorVoltage is not implemented");
  }

  default void setMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setMotorPercentage is not implemented");
  }

  default void setMotorRPM(double rpm) {
    throw new UnsupportedOperationException("setMotorRPM is not implemented");
  }

  default void setMotorPosition(double setpoint) {
    throw new UnsupportedOperationException("setMotorPosition is not implemented");
  }

  public class HopperIOInputs {
    
    /** Is the motor connected? */
    @Getter @Setter private boolean motorConnected;

     /** Motor data */
    @Getter @Setter private double motorVoltage;

    @Getter @Setter private double motorDutyCycle;
    @Getter @Setter private double motorCurrent;
    @Getter @Setter private double motorTemperature;
    @Getter @Setter private double motorVelocity;
    @Getter @Setter private double motorPosition;
  }

  default void updateInputs(HopperIOInputs hopperInputs) {}
}
