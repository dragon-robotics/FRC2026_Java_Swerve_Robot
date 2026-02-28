package frc.robot.util;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Telemetry {
  /**
   * Construct a telemetry object.
   */
  public Telemetry() {
    // SignalLogger.start();
  }

  /** Accept the swerve drive state and telemeterize it to DogLog. */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the swerve drive state */
    DogLog.log("Pose", state.Pose);
    DogLog.log("Speeds", state.Speeds);
    DogLog.log("ModuleStates", state.ModuleStates);
    DogLog.log("ModuleTargets", state.ModuleTargets);
    DogLog.log("ModulePositions", state.ModulePositions);
    DogLog.log("Timestamp", state.Timestamp);
    DogLog.log("OdometryFrequency", 1.0 / state.OdometryPeriod);

    // DogLog.log(
    //     "FinalIntakePose",
    //     new Pose3d[] {
    //       new Pose3d(
    //           0.2949,
    //           0,
    //           0.20099,
    //           // new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1, 0.0),
    //           new Rotation3d(0.0, Units.degreesToRadians(-125), 0.0))
    //     });
    // DogLog.log(
    //     "FinalExtendingHopperPose",
    //     new Pose3d[] {
    //       new Pose3d(
    //           // 0.05 + (((Math.sin(Timer.getTimestamp()) + 1) / 2) * Units.inchesToMeters(12.562)),
    //           0.05, 0.06375, 0.3, new Rotation3d(0.0, 0.0, 0.0))
    //     });
  }
}
