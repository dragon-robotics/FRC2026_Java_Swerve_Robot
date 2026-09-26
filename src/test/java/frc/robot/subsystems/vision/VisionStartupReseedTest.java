package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.Utils;
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
import frc.robot.subsystems.vision.VisionIO.FrameDiagnostic;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.constants.VisionConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VisionStartupReseedTest {
  private static final Pose2d INITIAL_POSE = new Pose2d(3, 3, Rotation2d.kZero);
  private static final Pose2d VISION_POSE = new Pose2d(4, 4, Rotation2d.fromDegrees(15));
  private static CommandSwerveDrivetrain swerve;

  private final TestCamera front = new TestCamera("startup-front");
  private final TestCamera rear = new TestCamera("startup-rear");
  private final List<Pose2d> accepted = new ArrayList<>();
  private final List<String> diagnostics = new ArrayList<>();
  private final Map<String, String> telemetry = new HashMap<>();
  private final List<double[]> measurements = new ArrayList<>();
  private boolean delayFifthObservation;
  private VisionSubsystem vision;

  @BeforeAll
  static void startDrivetrain() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.resumeTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    swerve = TunerConstants.createDrivetrain();
  }

  @BeforeEach
  void startVision() {
    // Observe ordinary fusion separately so a hard reset is visible in the native pose.
    vision =
        new VisionSubsystem(
            swerve,
            (pose, timestamp, sigma) -> {
              accepted.add(pose);
              measurements.add(
                  new double[] {timestamp, sigma.get(0, 0), sigma.get(1, 0), sigma.get(2, 0)});
            },
            (key, value) -> {
              telemetry.put(key, value);
              if (key.endsWith("/Observation")) diagnostics.add(value);
              if (delayFifthObservation && accepted.size() == 5) {
                delayFifthObservation = false;
                Timer.delay(.55);
              }
            },
            front,
            rear);
    vision.periodic();
    swerve.resetPose(INITIAL_POSE);
    captureTime();
  }

  @AfterEach
  void stopVision() {
    CommandScheduler.getInstance().unregisterSubsystem(vision);
  }

  @AfterAll
  static void stopDrivetrain() {
    if (swerve != null) swerve.close();
  }

  @Test
  void reportsActualStrategyAndAcceptanceIndependentlyForEachCamera() {
    double capture = captureTime();
    front.observation =
        new PoseObservation(
            capture,
            new Pose3d(VISION_POSE),
            .05,
            2,
            2,
            PoseObservationType.PHOTONVISION,
            new int[] {2, 3},
            "CONSTRAINED_SOLVEPNP",
            1);
    rear.observation =
        new PoseObservation(
            capture,
            new Pose3d(VISION_POSE),
            .9,
            1,
            2,
            PoseObservationType.PHOTONVISION,
            new int[] {2},
            "LOWEST_AMBIGUITY",
            1);
    vision.periodic();

    assertEquals("CONSTRAINED_SOLVEPNP", telemetry.get("Vision/startup-front/CurrentStrategy"));
    assertEquals("ACCEPTED", telemetry.get("Vision/startup-front/StrategyStatus"));
    assertEquals("LOWEST_AMBIGUITY", telemetry.get("Vision/startup-rear/CurrentStrategy"));
    assertEquals("AMBIGUITY=0.9", telemetry.get("Vision/startup-rear/StrategyStatus"));
    assertEquals(1, accepted.size(), "Reporting a rejected solver must not submit its pose");
  }

  @Test
  void newerNoTargetFrameClearsStrategyEvenWhenAnOlderPoseIsAccepted() {
    double capture = captureTime();
    front.observation =
        new PoseObservation(
            capture,
            new Pose3d(VISION_POSE),
            .05,
            2,
            2,
            PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            new int[] {2, 3},
            "MULTI_TAG_PNP_ON_COPROCESSOR",
            1);
    front.frames =
        new FrameDiagnostic[] {
          new FrameDiagnostic(capture + .001, 2, "NO_TARGETS", "NONE", new int[0])
        };
    vision.periodic();

    assertEquals(1, accepted.size());
    assertEquals("NONE", telemetry.get("Vision/startup-front/CurrentStrategy"));
    assertEquals("NO_TARGETS", telemetry.get("Vision/startup-front/StrategyStatus"));
  }

  @Test
  void strategyHoldsBetweenFramesButClearsOnStalenessAndDisconnection() {
    publish(front, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
    String strategy = telemetry.get("Vision/startup-front/CurrentStrategy");
    assertEquals("PHOTONVISION_MULTITAG_COPROCESSOR", strategy);
    vision.periodic();
    assertEquals(strategy, telemetry.get("Vision/startup-front/CurrentStrategy"));
    assertEquals("ACCEPTED", telemetry.get("Vision/startup-front/StrategyStatus"));

    Timer.delay(.55);
    vision.periodic();
    assertEquals("NONE", telemetry.get("Vision/startup-front/CurrentStrategy"));
    assertEquals("STALE_FRAME", telemetry.get("Vision/startup-front/StrategyStatus"));

    publish(front, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
    front.connected = false;
    vision.periodic();
    assertEquals("NONE", telemetry.get("Vision/startup-front/CurrentStrategy"));
    assertEquals("DISCONNECTED", telemetry.get("Vision/startup-front/StrategyStatus"));
    front.connected = true;
    vision.periodic();
    assertEquals("NONE", telemetry.get("Vision/startup-front/CurrentStrategy"));
    assertEquals("NO_FRAMES", telemetry.get("Vision/startup-front/StrategyStatus"));
  }

  @Test
  void startupReseedRequiresFiveAcceptedCoprocessorPosesFromOneCamera() {
    for (int i = 0; i < 4; i++) {
      publish(front, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
      assertPose(INITIAL_POSE, "The first four accepted poses must not hard-reset odometry");
    }
    assertEquals(
        4, accepted.size(), "Frames must pass the real filter and reach the fusion consumer");

    publish(rear, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
    assertPose(INITIAL_POSE, "A fifth pose from another camera must not complete startup");

    publish(front, VISION_POSE, PoseObservationType.PHOTONVISION, 2, 3);
    publish(front, VISION_POSE, PoseObservationType.PHOTONVISION, 2);
    assertEquals(7, accepted.size());
    assertPose(INITIAL_POSE, "Fallback and single-tag poses must not complete startup");

    publish(
        front,
        new Pose2d(-1, 4, Rotation2d.kZero),
        PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
        2,
        3);
    assertEquals(7, accepted.size(), "Rejected poses must not count toward startup");
    assertPose(INITIAL_POSE, "A rejected fifth MultiTag pose must not trigger a reset");

    publish(front, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
    assertEquals(8, accepted.size());
    assertPose(VISION_POSE, "The fifth accepted stable MultiTag pose from front must reseed");
  }

  @Test
  void forwardsEachCameraMeasurementInsteadOfSelectingOneWinner() {
    double capture = captureTime();
    front.observation =
        new PoseObservation(
            capture,
            new Pose3d(VISION_POSE),
            .05,
            2,
            2.0,
            PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            new int[] {2, 3});
    rear.observation =
        new PoseObservation(
            capture,
            new Pose3d(VISION_POSE),
            .05,
            2,
            2.0,
            PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            new int[] {2, 3});
    vision.periodic();
    assertEquals(2, accepted.size(), "Each independently valid camera must reach CTRE");
  }

  @Test
  void neverFusesTheSameCameraTimestampTwice() {
    double capture = captureTime();
    PoseObservation frame =
        new PoseObservation(
            capture,
            new Pose3d(VISION_POSE),
            .05,
            2,
            2.0,
            PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            new int[] {2, 3});
    front.observation = frame;
    vision.periodic();
    front.observation = frame;
    vision.periodic();
    assertEquals(
        1, accepted.size(), "Re-delivering a frame must not overweight it or advance startup");
  }

  @Test
  void startupReseedUsesTheCameraThatCompletedItsOwnFivePoseStreak() {
    for (int i = 0; i < 4; i++) {
      publish(front, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
    }
    double capture = captureTime();
    front.observation =
        new PoseObservation(
            capture,
            new Pose3d(VISION_POSE),
            .05,
            2,
            2.0,
            PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            new int[] {2, 3});
    Pose2d rearPose = new Pose2d(4.1, 4.1, Rotation2d.fromDegrees(17));
    rear.observation =
        new PoseObservation(
            capture + .002,
            new Pose3d(rearPose),
            .05,
            2,
            2.0,
            PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            new int[] {2, 3});
    vision.periodic();
    assertEquals(6, accepted.size());
    assertPose(
        VISION_POSE, "Rear's newer first frame must not replace front's qualifying fifth pose");
  }

  @Test
  void recordsAcceptanceAndDuplicateRejectionWithExactFusionArguments() throws Exception {
    double capture = captureTime();
    PoseObservation frame =
        new PoseObservation(
            capture,
            new Pose3d(VISION_POSE),
            .05,
            2,
            2.0,
            PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            new int[] {2, 3},
            "MULTI_TAG_PNP_ON_COPROCESSOR",
            42);
    front.observation = frame;
    vision.periodic();
    front.observation = frame;
    vision.periodic();
    assertEquals(2, diagnostics.size(), "Every decision needs its own complete record");
    JSONObject pass = (JSONObject) new JSONParser().parse(diagnostics.get(0));
    JSONObject reject = (JSONObject) new JSONParser().parse(diagnostics.get(1));
    assertEquals("startup-front", pass.get("camera"));
    assertEquals(42L, pass.get("frameSequenceId"));
    assertEquals(List.of(2L, 3L), pass.get("tagIds"));
    assertEquals(capture, (double) pass.get("captureTimestampSeconds"), 1e-9);
    assertEquals(measurements.get(0)[0], (double) pass.get("ctreTimestampSeconds"), 1e-9);
    assertEquals(Utils.fpgaToCurrentTime(capture), (double) pass.get("ctreTimestampSeconds"), .001);
    assertEquals(
        List.of(measurements.get(0)[1], measurements.get(0)[2], measurements.get(0)[3]),
        pass.get("suppliedStdDevs"));
    assertEquals(true, pass.get("accepted"));
    assertEquals(1L, pass.get("cameraStartupCount"));
    assertTrue(pass.get("referencePose") != null);
    assertEquals(false, reject.get("accepted"));
    assertEquals("DUPLICATE_OR_OUT_OF_ORDER", reject.get("rejectionReason"));
    assertEquals(null, reject.get("suppliedStdDevs"));
    assertEquals(1L, reject.get("cameraStartupCount"));
    assertEquals(1, accepted.size());
  }

  @Test
  void rejectsStaleFutureAndMissingHistoryWithoutAdvancingStartup() throws Exception {
    double capture = captureTime();
    for (double timestamp : new double[] {capture - 2, capture + 1}) {
      front.observation =
          new PoseObservation(
              timestamp,
              new Pose3d(VISION_POSE),
              .05,
              2,
              2.0,
              PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
              new int[] {2, 3});
      vision.periodic();
    }
    swerve.resetPose(INITIAL_POSE);
    front.observation =
        new PoseObservation(
            capture,
            new Pose3d(VISION_POSE),
            .05,
            2,
            2.0,
            PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            new int[] {2, 3});
    vision.periodic();
    assertEquals(0, accepted.size());
    String[] expected = {"STALE_FRAME", "FUTURE_TIMESTAMP", "NO_ODOMETRY_HISTORY"};
    for (int i = 0; i < expected.length; i++) {
      JSONObject data = (JSONObject) new JSONParser().parse(diagnostics.get(i));
      assertEquals(expected[i], data.get("rejectionReason"));
      assertEquals(0L, data.get("cameraStartupCount"));
    }
  }

  @Test
  void sixthUnstableFrameInTheBatchCannotReplaceTheQualifyingFifthPose() {
    for (int i = 0; i < 4; i++) {
      publish(front, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
    }
    double capture = captureTime();
    front.batch =
        new PoseObservation[] {
          new PoseObservation(
              capture,
              new Pose3d(VISION_POSE),
              .05,
              2,
              2,
              PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
              new int[] {2, 3}),
          new PoseObservation(
              capture + .001,
              new Pose3d(4.4, 4, 0, new edu.wpi.first.math.geometry.Rotation3d()),
              .05,
              2,
              2,
              PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
              new int[] {2, 3})
        };
    vision.periodic();
    assertEquals(6, accepted.size());
    assertPose(VISION_POSE, "Initial reset must use the pose which completed the stable streak");
  }

  @Test
  void cameraFactorsFollowNamesWhenTheBackCameraIsDisabled() {
    TestCamera left = new TestCamera("AprilTagPoseEstCameraL");
    List<Double> sigmas = new ArrayList<>();
    double backFactor = VisionConstants.CAMERA_STDDEV_FACTORS[2];
    double leftFactor = VisionConstants.CAMERA_STDDEV_FACTORS[3];
    VisionSubsystem other = null;
    try {
      VisionConstants.CAMERA_STDDEV_FACTORS[2] = 9;
      VisionConstants.CAMERA_STDDEV_FACTORS[3] = 3;
      other =
          new VisionSubsystem(
              swerve,
              (pose, timestamp, sigma) -> sigmas.add(sigma.get(0, 0)),
              new TestCamera("AprilTagPoseEstCameraF"),
              new TestCamera("AprilTagPoseEstCameraR"),
              left);
      left.observation =
          new PoseObservation(
              captureTime(),
              new Pose3d(VISION_POSE),
              .05,
              2,
              2,
              PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
              new int[] {2, 3});
      double unscaledSigma =
          VisionSubsystem.standardDeviations(left.observation, 0, false).get(0, 0);
      other.periodic();
      assertEquals(1, sigmas.size());
      assertEquals(
          3 * unscaledSigma,
          sigmas.get(0),
          1e-9,
          "Active L camera must use L's factor, not absent B's factor");
    } finally {
      VisionConstants.CAMERA_STDDEV_FACTORS[2] = backFactor;
      VisionConstants.CAMERA_STDDEV_FACTORS[3] = leftFactor;
      if (other != null) CommandScheduler.getInstance().unregisterSubsystem(other);
    }
  }

  @Test
  void expiredQualifyingSnapshotRequiresANewStreakAndCanRecover() {
    delayFifthObservation = true;
    for (int i = 0; i < 5; i++) {
      publish(front, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
    }
    assertPose(INITIAL_POSE, "An aged startup snapshot must not hard-reset odometry");
    for (int i = 0; i < 4; i++) {
      publish(front, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
      assertPose(INITIAL_POSE, "Expired qualification requires another five stable observations");
    }
    publish(front, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
    assertPose(VISION_POSE, "Fresh stable observations must recover startup localization");
  }

  @Test
  void unstableAcceptedPoseRestartsTheFivePoseRequirement() {
    for (int i = 0; i < 4; i++) {
      publish(front, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
      assertPose(INITIAL_POSE, "Startup must wait for five stable poses");
    }

    Pose2d shifted = new Pose2d(4.4, 4, Rotation2d.fromDegrees(15));
    for (int i = 0; i < 4; i++) {
      publish(front, shifted, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
      assertPose(INITIAL_POSE, "A 40 cm change must start a new stable streak");
    }
    assertEquals(8, accepted.size(), "The unstable step was accepted but must restart the streak");

    publish(front, shifted, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
    assertPose(shifted, "Five stable poses at the new location must allow startup reseeding");
  }

  @Test
  void pitchAndRollRejectBothSignsAndAllowRecovery() {
    var pigeon = swerve.getPigeon2();
    try {
      for (boolean pitch : new boolean[] {true, false}) {
        for (double tilt : new double[] {-9, 9}) {
          pigeon.getSimState().setPitch(pitch ? tilt : 0);
          pigeon.getSimState().setRoll(pitch ? 0 : tilt);
          pigeon.getPitch().waitForUpdate(.1);
          pigeon.getRoll().waitForUpdate(.1);
          publish(front, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
          assertEquals(0, accepted.size(), "Tilted robot must not fuse a pose or qualify startup");
          assertTrue(diagnostics.get(diagnostics.size() - 1).contains("TILT_UNSTABLE"));
        }
      }
    } finally {
      pigeon.getSimState().setPitch(0);
      pigeon.getSimState().setRoll(0);
      pigeon.getPitch().waitForUpdate(.1);
      pigeon.getRoll().waitForUpdate(.1);
    }
    for (int i = 0; i < 4; i++) {
      publish(front, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
      assertPose(INITIAL_POSE, "Tilt-rejected frames must not count toward startup");
    }
    publish(front, VISION_POSE, PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR, 2, 3);
    assertPose(VISION_POSE, "Level robot must recover with five accepted poses");
  }

  private void publish(TestCamera camera, Pose2d pose, PoseObservationType type, int... tagIds) {
    double capture = captureTime();
    camera.observation =
        new PoseObservation(capture, new Pose3d(pose), .05, tagIds.length, 2.0, type, tagIds);
    vision.periodic();
  }

  private double captureTime() {
    Timer.delay(.02);
    double deadline = Timer.getFPGATimestamp() + 3.0;
    do {
      double capture = Timer.getFPGATimestamp() - .03;
      if (swerve.samplePoseAt(capture).isPresent()) return capture;
      Timer.delay(.01);
    } while (Timer.getFPGATimestamp() < deadline);
    throw new AssertionError("Native simulation did not produce usable capture history");
  }

  private void assertPose(Pose2d expected, String message) {
    Pose2d actual = swerve.getState().Pose;
    // Native module simulation may settle by centimeters; a hard reset here moves over one meter.
    assertEquals(expected.getX(), actual.getX(), .03, message);
    assertEquals(expected.getY(), actual.getY(), .03, message);
    assertEquals(
        expected.getRotation().getDegrees(), actual.getRotation().getDegrees(), .1, message);
  }

  private static class TestCamera implements VisionIO {
    private final String name;
    private PoseObservation observation;
    private PoseObservation[] batch;
    private FrameDiagnostic[] frames = new FrameDiagnostic[0];
    private boolean connected = true;

    TestCamera(String name) {
      this.name = name;
    }

    @Override
    public String getCameraName() {
      return name;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.setCameraName(name);
      inputs.setConnected(connected);
      inputs.setFrameDiagnostics(frames);
      inputs.setPoseObservations(
          batch != null
              ? batch
              : observation == null ? new PoseObservation[0] : new PoseObservation[] {observation});
      observation = null;
      batch = null;
      frames = new FrameDiagnostic[0];
    }
  }
}
