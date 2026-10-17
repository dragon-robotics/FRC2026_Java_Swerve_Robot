package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VisionRecoveryTest {
  private static TestDrivetrain swerve;
  private final List<Pose2d> measurements = new ArrayList<>();
  private final TestCamera camera = new TestCamera();
  private VisionSubsystem vision;

  @BeforeAll
  static void initializeRobot() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    swerve = new TestDrivetrain();
  }

  @BeforeEach
  void prepareVision() {
    CommandScheduler.getInstance().unregisterAllSubsystems();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    swerve.resetPose(new Pose2d(1.0, 1.0, Rotation2d.kZero));
    vision =
        new VisionSubsystem(swerve, (pose, timestamp, stdDevs) -> measurements.add(pose), camera);
  }

  @AfterAll
  static void shutdownRobot() {
    CommandScheduler.getInstance().unregisterAllSubsystems();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
    HAL.shutdown();
  }

  @Test
  void enabledVisionCanCorrectMoreThanTwoPointFiveMetersOfOdometryError() {
    Pose2d actualPose = new Pose2d(6.0, 4.0, Rotation2d.fromDegrees(35));
    camera.observations = new PoseObservation[] {observation(actualPose)};

    vision.periodic();

    assertEquals(List.of(actualPose), measurements, "Large odometry error must not block fusion");
    assertEquals(actualPose, vision.getLatestAcceptedObservationSnapshot().orElseThrow().pose());
  }

  @Test
  void operatorResetUsesOneLatestFreshAcceptedPoseIncludingHeading() {
    Pose2d firstPose = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(10));
    camera.observations = new PoseObservation[] {observation(firstPose)};
    vision.periodic();
    SimHooks.stepTiming(0.02);
    Pose2d newestPose = new Pose2d(6.0, 4.0, Rotation2d.fromDegrees(75));
    camera.observations = new PoseObservation[] {observation(newestPose)};
    vision.periodic();

    assertTrue(vision.forceReseedFromVision());
    assertPose(newestPose);
  }

  @Test
  void operatorResetRejectsSnapshotOlderThanHalfASecond() {
    camera.observations =
        new PoseObservation[] {observation(new Pose2d(2.0, 1.0, Rotation2d.kZero))};
    vision.periodic();
    SimHooks.stepTiming(0.51);
    int resetsBeforeRequest = swerve.resetCount;

    assertFalse(vision.forceReseedFromVision());
    assertEquals(resetsBeforeRequest, swerve.resetCount);
  }

  @Test
  void operatorResetWithoutAcceptedVisionDoesNothing() {
    assertFalse(vision.forceReseedFromVision());
    assertPose(new Pose2d(1.0, 1.0, Rotation2d.kZero));
  }

  @Test
  void invalidVisionCannotBecomeAResetSnapshot() {
    camera.observations =
        new PoseObservation[] {observation(new Pose2d(-1.0, 1.0, Rotation2d.kZero))};
    vision.periodic();

    assertTrue(measurements.isEmpty());
    assertFalse(vision.forceReseedFromVision());
    assertPose(new Pose2d(1.0, 1.0, Rotation2d.kZero));
  }

  @Test
  void consensusCanSelectAgreementFarFromOdometryOverANearbyOutlier() {
    Pose2d agreedPose = new Pose2d(6.0, 4.0, Rotation2d.kZero);
    camera.observations =
        new PoseObservation[] {observation(new Pose2d(1.1, 1.0, Rotation2d.kZero))};
    TestCamera second = new TestCamera();
    TestCamera third = new TestCamera();
    second.observations = new PoseObservation[] {observation(agreedPose)};
    third.observations = new PoseObservation[] {observation(agreedPose)};
    vision =
        new VisionSubsystem(
            swerve, (pose, timestamp, stdDevs) -> measurements.add(pose), camera, second, third);

    vision.periodic();

    assertEquals(List.of(agreedPose), measurements);
  }

  private static PoseObservation observation(Pose2d pose) {
    return new PoseObservation(
        Timer.getFPGATimestamp(),
        new Pose3d(pose),
        0.05,
        2,
        2.0,
        PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
        new int[] {1, 2});
  }

  private static void assertPose(Pose2d expected) {
    Pose2d actual = swerve.getState().Pose;
    assertEquals(expected.getX(), actual.getX(), 1e-6);
    assertEquals(expected.getY(), actual.getY(), 1e-6);
    assertEquals(expected.getRotation().getRadians(), actual.getRotation().getRadians(), 1e-6);
  }

  private static class TestCamera implements VisionIO {
    private PoseObservation[] observations = new PoseObservation[0];

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.setCameraName("recovery-test");
      inputs.setConnected(true);
      inputs.setPoseObservations(observations);
    }
  }

  // Real drivetrain reset/readback, with deterministic odometry sampling and level attitude.
  private static class TestDrivetrain extends CommandSwerveDrivetrain {
    private int resetCount;

    TestDrivetrain() {
      super(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);
    }

    @Override
    public void resetPose(Pose2d pose) {
      resetCount++;
      super.resetPose(pose);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestamp) {
      return Optional.of(getState().Pose);
    }

    @Override
    public double getPitchDegrees() {
      return 0.0;
    }

    @Override
    public double getRollDegrees() {
      return 0.0;
    }
  }
}
