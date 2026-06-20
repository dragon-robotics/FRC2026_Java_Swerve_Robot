package frc.robot.io;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.units.measure.Voltage;
import lombok.Getter;
import lombok.Setter;

/**
 * Common motor-controller abstraction used by subsystems.
 *
 * <p>Methods are optional because not every controller supports every control mode. Callers should
 * only use methods supported by the concrete IO implementation or a narrower capability interface.
 */
public interface MotorIO {

  /** Directly commands motor voltage. */
  default void setMotorVoltage(Voltage voltage) {
    throw new UnsupportedOperationException("setMotorVoltage not implemented");
  }

  /** Directly commands percent output from -1.0 to 1.0. */
  default void setMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setMotorPercentage not implemented");
  }

  /** Commands motor velocity in RPM. */
  default void setMotorRPM(double rpm) {
    throw new UnsupportedOperationException("setMotorRPM not implemented");
  }

  /** Commands motor position. Units depend on the configured sensor, usually rotations. */
  default void setMotorPosition(double setpoint) {
    throw new UnsupportedOperationException("setMotorPosition is not implemented");
  }

  /**
   * Commands motor position with a controller slot.
   *
   * @param setpoint position setpoint, usually mechanism rotations
   * @param slotID closed-loop slot index
   */
  default void setMotorPosition(double setpoint, int slotID) {
    throw new UnsupportedOperationException("setMotorPosition is not implemented");
  }

  /** Snapshot of telemetry read from one motor controller. */
  class MotorIOInputs {
    /** True when the backing controller/status signals report healthy communication. */
    @Getter @Setter private boolean motorConnected;

    /** Position in the configured sensor's native mechanism units, usually rotations. */
    @Getter @Setter private double motorPosition;

    /** Velocity in controller-reported units; CTRE values are rotations per second. */
    @Getter @Setter private double motorVelocity;

    /** Applied motor voltage. */
    @Getter @Setter private double motorVoltage;

    /** Motor current in amps. */
    @Getter @Setter private double motorCurrent;

    /** Motor temperature in degrees Celsius. */
    @Getter @Setter private double motorTemperature;
  }

  /** Updates the provided telemetry snapshot. */
  default void updateInputs(MotorIOInputs inputs) {}

  /** Returns the user-friendly name for this motor. */
  default String getMotorName() {
    return "Unknown";
  }

  /**
   * Returns CTRE status signals to batch-refresh each robot loop.
   *
   * <p>Non-CTRE implementations return an empty array.
   */
  default BaseStatusSignal[] getStatusSignals() {
    return new BaseStatusSignal[0];
  }
}
