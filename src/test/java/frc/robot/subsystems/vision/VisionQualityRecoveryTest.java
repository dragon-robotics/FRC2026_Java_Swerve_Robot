package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import java.util.function.BooleanSupplier;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VisionQualityRecoveryTest {
  // Independent stationary field truth: never derive a camera observation from the fused estimate.
  private static final Pose2d TRUTH = new Pose2d(4, 3, Rotation2d.fromDegrees(35));
  private static final int RECOVERY_FRAMES = 50;
  private static CommandSwerveDrivetrain swerve;

  private final TestCamera camera = new TestCamera();
  private final List<JSONObject> decisions = new ArrayList<>();
  private VisionSubsystem vision;
  private int fusedFrames;

  @BeforeAll
  static void startDrivetrain() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.resumeTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    swerve = TunerConstants.createDrivetrain();
    assertFalse(
        swerve.isFieldHeadingAligned(), "The constructor's zero pose is not field alignment");
  }

  @BeforeEach
  void startVision() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    assertTrue(DriverStation.isEnabled(), "Recovery must run without disabled startup hard resets");
    vision =
        new VisionSubsystem(
            swerve,
            (pose, timestamp, sigma) -> {
              fusedFrames++;
              swerve.addVisionMeasurement(pose, timestamp, sigma);
            },
            (key, value) -> {
              if (key.endsWith("/Observation")) {
                try {
                  decisions.add((JSONObject) new JSONParser().parse(value));
                } catch (ParseException exception) {
                  throw new AssertionError("Invalid vision decision record", exception);
                }
              }
            },
            camera);
    vision.periodic();
  }

  @AfterEach
  void stopVision() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().unregisterSubsystem(vision);
  }

  @AfterAll
  static void stopDrivetrain() {
    if (swerve != null) swerve.close();
  }

  @Test
  void nativeFusionRecoversTranslationOffsetsAfterTemporaryVisionLoss() {
    for (double offset : new double[] {.25, .5, 1, 2}) {
      Pose2d initial = offsetPose(offset);
      swerve.resetPose(initial, "RECOVERY_TEST_KNOWN_HEADING");
      captureTime();
      int beforeRecovery = fusedFrames;

      camera.connected = false;
      // Exceed the accepted-snapshot lifetime while native odometry continues running.
      for (int i = 0; i < 30; i++) {
        Timer.delay(.02);
        vision.periodic();
      }
      assertEquals(
          beforeRecovery, fusedFrames, "Vision loss must not submit retained observations");
      assertTrue(
          swerve.isFieldHeadingAligned(), "Camera loss does not invalidate the field heading");
      assertTrue(
          swerve.getState().Pose.getTranslation().getDistance(initial.getTranslation()) < .04,
          "No measurement or disabled startup reset may erase the deliberate offset");

      camera.connected = true;
      for (int i = 0; i < RECOVERY_FRAMES; i++) publish(TRUTH);
      assertEquals(
          RECOVERY_FRAMES,
          fusedFrames - beforeRecovery,
          "Every fresh quality frame must remain usable for " + offset + " m recovery");
      waitUntil(() -> translationError() < .06, "Native fusion did not recover " + offset + " m");
      assertTrue(
          translationError() < offset * .25, "Fusion must remove at least 75% of the offset");
      assertEquals(
          0,
          swerve.getState().Pose.getRotation().minus(TRUTH.getRotation()).getDegrees(),
          .1,
          "Ordinary fusion must preserve the established gyro heading");
      System.out.printf(
          "Vision recovery: initial=%.2f m, final=%.4f m, accepted=%d/%d%n",
          offset, translationError(), fusedFrames - beforeRecovery, RECOVERY_FRAMES);
    }
  }

  @Test
  void inconsistentHeadingCannotFuseBeforeCleanFramesRecoverNativePose() {
    Pose2d initial = offsetPose(1);
    swerve.resetPose(initial, "RECOVERY_TEST_KNOWN_HEADING");
    Pose2d wrongHeading =
        new Pose2d(TRUTH.getTranslation(), TRUTH.getRotation().plus(Rotation2d.fromDegrees(12)));
    for (int i = 0; i < 10; i++) publish(wrongHeading);

    assertEquals(
        0, fusedFrames, "Heading-inconsistent observations must never reach native fusion");
    assertEquals(10, decisions.size());
    for (JSONObject decision : decisions) {
      assertEquals("HEADING_DELTA", decision.get("rejectionReason"));
      assertEquals(true, decision.get("headingGateActive"));
    }
    assertTrue(
        swerve.getState().Pose.getTranslation().getDistance(initial.getTranslation()) < .04,
        "Rejected observations must leave the deliberately offset native estimate unchanged");

    for (int i = 0; i < RECOVERY_FRAMES; i++) publish(TRUTH);
    assertEquals(
        RECOVERY_FRAMES, fusedFrames, "Rejection must not lock out subsequent clean frames");
    waitUntil(
        () -> translationError() < .06, "Clean frames did not recover after heading rejection");
  }

  @Test
  void onlyAbsoluteHeadingResetsEstablishAlignment() {
    swerve.tareEverything();
    assertFalse(swerve.isFieldHeadingAligned());
    swerve.resetTranslation(new Translation2d(2, 3));
    assertFalse(swerve.isFieldHeadingAligned(), "Translation cannot establish a heading reference");
    swerve.setOperatorPerspectiveForward(Rotation2d.k180deg);
    assertFalse(
        swerve.isFieldHeadingAligned(), "Operator perspective is not absolute pose alignment");

    swerve.resetRotation(TRUTH.getRotation());
    assertTrue(swerve.isFieldHeadingAligned());
    swerve.resetTranslation(TRUTH.getTranslation());
    assertTrue(swerve.isFieldHeadingAligned(), "Translation must preserve an established heading");
    swerve.setOperatorPerspectiveForward(Rotation2d.kZero);
    assertTrue(
        swerve.isFieldHeadingAligned(), "Changing driver perspective must preserve alignment");

    double beforeSeed = captureTime();
    swerve.seedFieldCentric();
    assertFalse(swerve.isFieldHeadingAligned(), "Arbitrary operator-forward is not field heading");
    assertTrue(swerve.samplePoseAt(beforeSeed).isEmpty());
    swerve.resetPose(TRUTH);
    assertTrue(swerve.isFieldHeadingAligned());
    double beforeTare = captureTime();
    swerve.tareEverything();
    assertFalse(swerve.isFieldHeadingAligned());
    assertTrue(swerve.samplePoseAt(beforeTare).isEmpty());
  }

  private static Pose2d offsetPose(double distance) {
    // 0.6/0.8 yields a unit vector, exercising both X and Y with the requested total offset.
    return new Pose2d(
        TRUTH.getX() + distance * .6, TRUTH.getY() + distance * .8, TRUTH.getRotation());
  }

  private void publish(Pose2d measuredPose) {
    camera.observation =
        new PoseObservation(
            captureTime(),
            new Pose3d(measuredPose),
            .05,
            2,
            2,
            PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            new int[] {2, 3});
    vision.periodic();
  }

  private static double captureTime() {
    Timer.delay(.02);
    double[] capture = new double[1];
    waitUntil(
        () -> {
          capture[0] = Timer.getFPGATimestamp() - .03;
          return swerve.samplePoseAt(capture[0]).isPresent();
        },
        "Native simulation did not produce usable capture history");
    return capture[0];
  }

  private static double translationError() {
    return swerve.getState().Pose.getTranslation().getDistance(TRUTH.getTranslation());
  }

  private static void waitUntil(BooleanSupplier condition, String message) {
    long deadline = System.nanoTime() + 3_000_000_000L;
    while (!condition.getAsBoolean() && System.nanoTime() < deadline) Timer.delay(.005);
    assertTrue(
        condition.getAsBoolean(), () -> message + "; current pose=" + swerve.getState().Pose);
  }

  private static class TestCamera implements VisionIO {
    private PoseObservation observation;
    private boolean connected = true;

    @Override
    public String getCameraName() {
      return "quality-recovery";
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.setCameraName(getCameraName());
      inputs.setConnected(connected);
      inputs.setPoseObservations(
          observation == null ? new PoseObservation[0] : new PoseObservation[] {observation});
      observation = null;
    }
  }
}
