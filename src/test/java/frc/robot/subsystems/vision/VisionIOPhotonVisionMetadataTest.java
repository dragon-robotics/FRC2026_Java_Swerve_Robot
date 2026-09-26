package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIO.FrameDiagnostic;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;
import org.photonvision.targeting.TargetCorner;

class VisionIOPhotonVisionMetadataTest {
  private VisionIOPhotonVision io;
  private final VisionIOInputs inputs = new VisionIOInputs();
  private double angularRate;
  private boolean headingAvailable = true;
  private String previousStrategyOrder;
  private String previousStrategyMode;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void createCamera() {
    previousStrategyOrder = System.getProperty("vision.photon.strategyOrder");
    previousStrategyMode = System.getProperty("vision.photon.strategyMode");
    System.clearProperty("vision.photon.strategyOrder");
    System.clearProperty("vision.photon.strategyMode");
    io = new VisionIOPhotonVision("metadata-test", new Transform3d());
  }

  @AfterEach
  void closeCamera() {
    io.camera.close();
    restoreProperty("vision.photon.strategyOrder", previousStrategyOrder);
    restoreProperty("vision.photon.strategyMode", previousStrategyMode);
  }

  @Test
  void coprocessorPoseUsesOnlyIdsReportedByItsSolve() throws Exception {
    var observations =
        process(
            frame(
                List.of(target(2, 2.0, 0.1), target(3, 4.0, 0.2), target(4, 6.0, 0.3)),
                List.of((short) 2, (short) 3)));

    assertEquals(1, observations.size());
    PoseObservation observation = observations.get(0);
    assertArrayEquals(new int[] {2, 3}, observation.tagIDs());
    assertEquals(2, observation.tagCount());
    assertEquals(3.0, observation.averageTagDistance(), 1e-9);
    assertEquals(0.15, observation.ambiguity(), 1e-9);
    assertEquals("MULTI_TAG_PNP_ON_COPROCESSOR", observation.solver());
    assertEquals(42, observation.frameSequenceId());
  }

  @Test
  void lowestAmbiguityFallbackUsesOneTargetDespiteOtherVisibleTags() throws Exception {
    var observations =
        process(
            frame(List.of(target(2, 5.0, 0.3), target(3, 2.0, 0.05), target(4, 1.0, -1.0)), null));

    assertEquals(1, observations.size());
    assertArrayEquals(new int[] {3}, observations.get(0).tagIDs());
    assertEquals(1, observations.get(0).tagCount());
    assertEquals(2.0, observations.get(0).averageTagDistance(), 1e-9);
    assertEquals(0.05, observations.get(0).ambiguity(), 1e-9);
    assertEquals("LOWEST_AMBIGUITY", observations.get(0).solver());
  }

  @Test
  void missingCoprocessorContributorRejectsWholeSolvedPose() throws Exception {
    assertTrue(
        process(frame(List.of(target(2, 2.0, 0.1)), List.of((short) 2, (short) 3))).isEmpty());
  }

  @Test
  void unknownCoprocessorContributorRejectsWholeSolvedPose() throws Exception {
    assertTrue(
        process(
                frame(
                    List.of(target(2, 2.0, 0.1), target(999, 2.0, 0.1)),
                    List.of((short) 2, (short) 999)))
            .isEmpty());
  }

  @Test
  void defaultSingleTagUsesCaptureTimeHeadingAfterInitialization() throws Exception {
    provideZeroHeading();
    io.markVisionInitializationComplete();
    var observations = process(frame(List.of(target(2, 2.0, 0.1)), null));

    assertEquals(1, observations.size());
    assertEquals(0.0, observations.get(0).pose().getRotation().toRotation2d().getDegrees(), 1e-6);
    assertEquals("PNP_DISTANCE_TRIG_SOLVE", observations.get(0).solver());
  }

  @Test
  void explicitTrigComparisonUsesOnlyBestTarget() throws Exception {
    provideZeroHeading();
    io.markVisionInitializationComplete();
    System.setProperty("vision.photon.strategyOrder", "PNP_DISTANCE_TRIG_SOLVE");
    var observations = process(frame(List.of(target(2, 2.0, 0.3), target(3, 5.0, 0.05)), null));

    assertEquals(1, observations.size());
    assertArrayEquals(new int[] {2}, observations.get(0).tagIDs());
    assertEquals(1, observations.get(0).tagCount());
    assertEquals(2.0, observations.get(0).averageTagDistance(), 1e-9);
    assertEquals("PNP_DISTANCE_TRIG_SOLVE", observations.get(0).solver());
  }

  @Test
  void failedFramesKeepTheirIndividualDiagnosticMetadata() throws Exception {
    io.processResults(
        List.of(
            frame(List.of(), null),
            frame(List.of(target(2, 9.0, .1)), null),
            frame(List.of(target(2, 2.0, -1.0)), null),
            frame(List.of(target(2, 2.0, .1)), List.of((short) 2, (short) 3))),
        inputs);

    List<FrameDiagnostic> diagnostics = List.of(inputs.getFrameDiagnostics());
    assertEquals(4, diagnostics.size());
    assertEquals("NO_TARGETS", diagnostics.get(0).status());
    assertEquals("ALL_TARGETS_BEYOND_RANGE", diagnostics.get(1).status());
    assertEquals("NO_POSE", diagnostics.get(2).status());
    assertEquals("INVALID_SOLVE_METADATA", diagnostics.get(3).status());
    assertEquals("MULTI_TAG_PNP_ON_COPROCESSOR", diagnostics.get(3).solver());
    assertArrayEquals(new int[] {2}, diagnostics.get(3).visibleTagIDs());
    assertEquals(42, diagnostics.get(3).frameSequenceId());
    assertEquals(1.0, diagnostics.get(3).timestamp(), 1e-9);
  }

  @Test
  void missingOrRepeatedCoprocessorIdsCannotFabricateMultiTagSupport() throws Exception {
    assertTrue(
        process(frame(List.of(target(2, 2.0, 0.1), target(3, 3.0, 0.1)), List.of())).isEmpty());
    assertTrue(
        process(frame(List.of(target(2, 2.0, 0.1)), List.of((short) 2, (short) 2))).isEmpty());
  }

  @Test
  void negativeNonSentinelAmbiguityMatchesPhotonSelectionAndRemainsVisible() throws Exception {
    var observations = process(frame(List.of(target(2, 2.0, -2.0), target(3, 3.0, 0.1)), null));

    assertEquals(1, observations.size());
    assertArrayEquals(new int[] {2}, observations.get(0).tagIDs());
    assertEquals(-2.0, observations.get(0).ambiguity());
  }

  @ParameterizedTest
  @ValueSource(doubles = {-1.01, 1.01, Double.NaN, Double.POSITIVE_INFINITY})
  void unsafeAngularRateFallsBackToCameraOnlyPose(double rate) throws Exception {
    provideZeroHeading();
    angularRate = rate;
    io.markVisionInitializationComplete();
    assertEquals(
        "LOWEST_AMBIGUITY", process(frame(List.of(target(2, 2, .1)), null)).get(0).solver());
  }

  @ParameterizedTest
  @ValueSource(doubles = {-1.0, 1.0})
  void trigRemainsEligibleAtItsAngularRateBoundary(double rate) throws Exception {
    provideZeroHeading();
    angularRate = rate;
    io.markVisionInitializationComplete();
    assertEquals(
        "PNP_DISTANCE_TRIG_SOLVE", process(frame(List.of(target(2, 2, .1)), null)).get(0).solver());
  }

  @Test
  void missingHeadingHistoryFallsBackToCameraOnlyPose() throws Exception {
    provideZeroHeading();
    headingAvailable = false;
    io.markVisionInitializationComplete();
    assertEquals(
        "LOWEST_AMBIGUITY", process(frame(List.of(target(2, 2, .1)), null)).get(0).solver());
  }

  @Test
  void missingCalibrationFallsBackToCoprocessorMultiTag() throws Exception {
    provideZeroHeading();
    io.markVisionInitializationComplete();
    System.setProperty(
        "vision.photon.strategyOrder",
        "CONSTRAINED_SOLVEPNP,MULTI_TAG_PNP_ON_COPROCESSOR,LOWEST_AMBIGUITY");
    assertEquals(
        "MULTI_TAG_PNP_ON_COPROCESSOR",
        process(frame(List.of(target(2, 2, .1), target(3, 2, .1)), List.of((short) 2, (short) 3)))
            .get(0)
            .solver());
  }

  @Test
  void startupAndRestartOverrideExperimentalStrategyOrder() throws Exception {
    provideZeroHeading();
    System.setProperty("vision.photon.strategyOrder", "PNP_DISTANCE_TRIG_SOLVE");
    var frame = frame(List.of(target(2, 2, .1), target(3, 2, .1)), List.of((short) 2, (short) 3));
    var inputs = new VisionIOInputs();
    io.processResults(List.of(frame), inputs);
    assertEquals("MULTI_TAG_PNP_ON_COPROCESSOR", inputs.getPoseObservations()[0].solver());
    io.markVisionInitializationComplete();
    io.processResults(List.of(frame), inputs);
    assertEquals("PNP_DISTANCE_TRIG_SOLVE", inputs.getPoseObservations()[0].solver());
    io.restartVisionInitialization();
    io.processResults(List.of(frame), inputs);
    assertEquals("MULTI_TAG_PNP_ON_COPROCESSOR", inputs.getPoseObservations()[0].solver());
    io.processResults(List.of(), inputs);
    assertEquals(0, inputs.getPoseObservations().length);
    assertEquals(0, inputs.getFrameDiagnostics().length);
  }

  private void provideZeroHeading() {
    io.setHeadingProvider(
        new VisionIOPhotonVision.VisionHeadingProvider() {
          @Override
          public Optional<Rotation2d> getHeadingAtTimestamp(double timestamp) {
            return headingAvailable ? Optional.of(new Rotation2d()) : Optional.empty();
          }

          @Override
          public Optional<Pose3d> getSeedPoseAtTimestamp(double timestamp) {
            return Optional.empty();
          }

          @Override
          public double getAngularRateRadPerSec() {
            return angularRate;
          }

          @Override
          public double getLinearSpeedMetersPerSecond() {
            return 0.0;
          }
        });
  }

  private List<PoseObservation> process(PhotonPipelineResult result) {
    io.processResults(List.of(result), inputs);
    return List.of(inputs.getPoseObservations());
  }

  private static PhotonPipelineResult frame(List<PhotonTrackedTarget> targets, List<Short> ids) {
    return new PhotonPipelineResult(
        42,
        1_000_000,
        1_010_000,
        0,
        targets,
        ids == null
            ? Optional.empty()
            : Optional.of(
                new MultiTargetPNPResult(
                    new PnpResult(new Transform3d(2.0, 3.0, 0.0, new Rotation3d()), 0.2), ids)));
  }

  private static PhotonTrackedTarget target(int id, double distance, double ambiguity) {
    List<TargetCorner> corners =
        List.of(
            new TargetCorner(0, 0),
            new TargetCorner(1, 0),
            new TargetCorner(1, 1),
            new TargetCorner(0, 1));
    Transform3d transform = new Transform3d(distance, 0.0, 0.0, new Rotation3d());
    return new PhotonTrackedTarget(
        0, 0, 1, 0, id, -1, -1, transform, transform, ambiguity, corners, corners);
  }

  private static void restoreProperty(String name, String previousValue) {
    if (previousValue == null) {
      System.clearProperty(name);
    } else {
      System.setProperty(name, previousValue);
    }
  }
}
