package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.generated.TunerConstants;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class VisionCaptureHistoryTest {
  @Test
  void resetInvalidatesOldFramesAndConsumerKeepsCaptureTime() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.resumeTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    var swerve = TunerConstants.createDrivetrain();
    List<Pose2d> poses = new ArrayList<>();
    List<Double> timestamps = new ArrayList<>();
    var camera =
        new VisionIO() {
          PoseObservation observation;

          public String getCameraName() {
            return "history-test";
          }

          public void updateInputs(VisionIOInputs inputs) {
            inputs.setCameraName(getCameraName());
            inputs.setConnected(true);
            inputs.setPoseObservations(
                observation == null ? new PoseObservation[0] : new PoseObservation[] {observation});
          }
        };
    var vision =
        new VisionSubsystem(
            swerve,
            (pose, timestamp, sigma) -> {
              poses.add(pose);
              timestamps.add(timestamp);
            },
            camera);
    try {
      vision.periodic(); // Initialize logging before capturing the first age-limited frame.
      var pose = new Pose2d(3, 3, Rotation2d.kZero);
      swerve.resetPose(pose);
      Timer.delay(.03);
      assertTrue(
          swerve.samplePoseAt(Timer.getFPGATimestamp() - .02).isEmpty(),
          "First observed odometry update establishes a conservative lower history bound");
      Timer.delay(.08);
      double capture = Timer.getFPGATimestamp() - .03;
      assertTrue(swerve.samplePoseAt(capture).isPresent());
      camera.observation = observation(capture, new Pose2d(3.1, 3, Rotation2d.kZero));
      vision.periodic();
      assertEquals(1, poses.size());
      assertEquals(camera.observation.pose().toPose2d(), poses.get(0));
      assertEquals(Utils.fpgaToCurrentTime(capture), timestamps.get(0), .002);

      swerve.resetPose(pose);
      assertTrue(
          swerve.samplePoseAt(capture).isEmpty(),
          "Capture from before reset must not be clamped into new history");
      vision.periodic();
      assertEquals(1, poses.size(), "Pre-reset frame must not reach consumer");
      assertTrue(swerve.samplePoseAt(Timer.getFPGATimestamp() + .01).isEmpty());
      assertTrue(swerve.samplePoseAt(Double.NaN).isEmpty());
      assertTrue(swerve.samplePoseAt(Timer.getFPGATimestamp() - 2).isEmpty());

      Timer.delay(.03);
      swerve.samplePoseAt(Timer.getFPGATimestamp());
      Timer.delay(.08);
      capture = Timer.getFPGATimestamp() - .03;
      camera.observation = observation(capture, new Pose2d(6, 3, Rotation2d.kZero));
      vision.periodic();
      assertEquals(1, poses.size(), "Enabled large solo correction must be rejected");
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      vision.periodic();
      assertEquals(
          2, poses.size(), "Disabled manual localization must retain large-correction recovery");
      assertEquals(
          pose.getX(),
          swerve.getState().Pose.getX(),
          .01,
          "Automatic hard reseed must stay blocked after enable");
    } finally {
      edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().unregisterSubsystem(vision);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      swerve.close();
    }
  }

  private static VisionIO.PoseObservation observation(double time, Pose2d pose) {
    return new VisionIO.PoseObservation(
        time,
        new Pose3d(pose),
        .05,
        2,
        2,
        VisionIO.PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
        new int[] {2, 3});
  }
}
