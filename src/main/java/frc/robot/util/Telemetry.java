package frc.robot.util;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import dev.doglog.DogLog;

public class Telemetry {
  /**
   * Construct a telemetry object, with the specified max speed of the robot
   *
   * @param maxSpeed Maximum speed in meters per second
   */
  public Telemetry(double maxSpeed) {
    SignalLogger.start();
  }

  /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the swerve drive state */
    DogLog.log("Pose", state.Pose);
    DogLog.log("Speeds", state.Speeds);
    DogLog.log("ModuleStates", state.ModuleStates);
    DogLog.log("ModuleTargets", state.ModuleTargets);
    DogLog.log("ModulePositions", state.ModulePositions);
    DogLog.log("Timestamp", state.Timestamp);
    DogLog.log("OdometryFrequency", 1.0 / state.OdometryPeriod);
  }
}