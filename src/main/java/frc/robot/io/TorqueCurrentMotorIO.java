package frc.robot.io;

import edu.wpi.first.units.measure.Current;

/**
 * Optional motor IO capability for controllers that can command torque current.
 *
 * <p>This is separate from {@link MotorIO} because Spark MAX and other controllers may not support
 * this control mode.
 */
public interface TorqueCurrentMotorIO extends MotorIO {

  /** Commands torque current in amps. */
  void setMotorTorqueCurrent(Current torqueCurrent);
}
