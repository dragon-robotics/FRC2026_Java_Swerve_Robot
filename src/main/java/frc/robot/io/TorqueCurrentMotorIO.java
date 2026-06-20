package frc.robot.io;

import edu.wpi.first.units.measure.Current;

public interface TorqueCurrentMotorIO extends MotorIO {

  void setMotorTorqueCurrent(Current torqueCurrent);
}
