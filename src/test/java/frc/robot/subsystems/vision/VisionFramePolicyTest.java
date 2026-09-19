package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import java.lang.reflect.Method;
import java.util.*;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.*;

class VisionFramePolicyTest {
  @BeforeAll
  static void init() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void singleTargetSolversReportOnlyChosenTarget() throws Exception {
    for (String strategy : List.of("LOWEST_AMBIGUITY", "PNP_DISTANCE_TRIG_SOLVE")) {
      String previous = System.getProperty("vision.photon.strategyOrder");
      var io = new VisionIOPhotonVision("metadata-" + strategy, new Transform3d());
      try {
        System.setProperty("vision.photon.strategyOrder", strategy);
        io.markVisionInitializationComplete();
        io.setHeadingProvider(
            new VisionIOPhotonVision.VisionHeadingProvider() {
              public Optional<Rotation2d> getHeadingAtTimestamp(double t) {
                return Optional.of(new Rotation2d());
              }

              public Optional<Pose3d> getSeedPoseAtTimestamp(double t) {
                return Optional.of(new Pose3d());
              }

              public double getAngularRateRadPerSec() {
                return 0;
              }

              public double getLinearSpeedMetersPerSecond() {
                return 0;
              }
            });
        var result = frame(1.0, List.of(target(2, 0.3), target(3, 0.4), target(4, 0.5)));
        io.poseEstimator.addHeadingData(1.0, new Rotation2d());
        var raw =
            strategy.equals("LOWEST_AMBIGUITY")
                ? io.poseEstimator.estimateLowestAmbiguityPose(result)
                : io.poseEstimator.estimatePnpDistanceTrigSolvePose(result);
        assertTrue(raw.isPresent(), "Fixture must produce an actual PhotonLib solve");
        assertEquals(
            3, raw.get().targetsUsed.size(), "Installed PhotonLib exposes all visible tags");
        Method process =
            VisionIOPhotonVision.class.getDeclaredMethod(
                "processResult", PhotonPipelineResult.class, VisionIO.VisionIOInputs.class);
        process.setAccessible(true);
        process.invoke(io, result, new VisionIO.VisionIOInputs());
        var field = VisionIOPhotonVision.class.getDeclaredField("poseObservations");
        field.setAccessible(true);
        @SuppressWarnings("unchecked")
        var observations = (List<VisionIO.PoseObservation>) field.get(io);
        assertEquals(1, observations.size());
        var observation = observations.get(0);
        assertEquals(1, observation.tagCount(), strategy);
        assertArrayEquals(new int[] {2}, observation.tagIDs());
        assertEquals(0.3, observation.ambiguity(), 1e-9);
        assertTrue(VisionSubsystem.rejectionReason(observation).isPresent());
      } finally {
        io.camera.close();
        if (previous == null) System.clearProperty("vision.photon.strategyOrder");
        else System.setProperty("vision.photon.strategyOrder", previous);
      }
    }
  }

  @Test
  void queueIsBoundedBeforeSolvingAndDoesNotRetryOldFrames() {
    String previous = System.getProperty("vision.photon.strategyOrder");
    var io = new VisionIOPhotonVision("queue-policy", new Transform3d());
    try {
      System.setProperty("vision.photon.strategyOrder", "LOWEST_AMBIGUITY");
      io.markVisionInitializationComplete();
      var inputs = new VisionIO.VisionIOInputs();
      var good = List.of(target(2, 0.1));
      List<PhotonPipelineResult> backlog = new ArrayList<>();
      for (int i = 0; i < 100; i++) backlog.add(frame(1.0 + i * 0.009, good));
      backlog.add(frame(1.99, good));
      backlog.add(frame(2.10, good)); // Future timestamp must not suppress valid latest frame.
      Collections.shuffle(backlog, new Random(42));
      io.processUnreadResults(backlog, inputs, 2.0);
      assertEquals(1, inputs.getPoseObservations().length);
      assertEquals(1.99, inputs.getPoseObservations()[0].timestamp(), 1e-9);
      io.processUnreadResults(backlog, inputs, 2.0);
      assertEquals(
          0, inputs.getPoseObservations().length, "Repeated frames must not be fused twice");
      io.processUnreadResults(List.of(frame(1.98, good)), inputs, 2.0);
      assertEquals(0, inputs.getPoseObservations().length, "Out-of-order frames must not be fused");
      io.processUnreadResults(
          List.of(frame(2.0, good), frame(2.01, List.of(target(999, 0.1)))), inputs, 2.02);
      assertEquals(
          0,
          inputs.getPoseObservations().length,
          "Failed newest solve must not retry older frames");
      io.processUnreadResults(List.of(frame(2.03, good), frame(2.04, List.of())), inputs, 2.05);
      assertEquals(
          0,
          inputs.getPoseObservations().length,
          "Newest empty frame must suppress older detections");
      io.processUnreadResults(List.of(frame(2.03, good)), inputs, 2.05);
      assertEquals(
          0,
          inputs.getPoseObservations().length,
          "Empty frame must advance the timestamp watermark");
      assertFalse(VisionIOPhotonVision.isFreshTimestamp(1.0, 2.0));
      assertFalse(VisionIOPhotonVision.isFreshTimestamp(Double.NaN, 2.0));
    } finally {
      io.camera.close();
      if (previous == null) System.clearProperty("vision.photon.strategyOrder");
      else System.setProperty("vision.photon.strategyOrder", previous);
    }
  }

  @Test
  void coprocessorConfidenceUsesItsActualTagIds() {
    var targets = List.of(target(2, 0.1), target(3, 0.1), target(4, 0.1));
    var multi = new MultiTargetPNPResult();
    multi.fiducialIDsUsed = List.of((short) 2, (short) 3);
    var frame = new PhotonPipelineResult(1, 1_000_000, 1_010_000, 0, targets, Optional.of(multi));
    assertEquals(
        List.of(targets.get(0), targets.get(1)),
        VisionIOPhotonVision.targetsUsedByStrategy(
            frame, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
    var differentBest = frame(1.0, List.of(target(2, 0.4), target(3, 0.05), target(4, 0.2)));
    assertEquals(
        3,
        VisionIOPhotonVision.targetsUsedByStrategy(differentBest, PoseStrategy.LOWEST_AMBIGUITY)
            .get(0)
            .getFiducialId());
    assertEquals(
        2,
        VisionIOPhotonVision.targetsUsedByStrategy(
                differentBest, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE)
            .get(0)
            .getFiducialId());
  }

  static PhotonPipelineResult frame(double timestamp, List<PhotonTrackedTarget> targets) {
    long micros = (long) (timestamp * 1_000_000);
    return new PhotonPipelineResult(1, micros, micros + 10_000, 0, targets);
  }

  static PhotonTrackedTarget target(int id, double ambiguity) {
    var corners =
        List.of(
            new TargetCorner(0, 0),
            new TargetCorner(1, 0),
            new TargetCorner(1, 1),
            new TargetCorner(0, 1));
    return new PhotonTrackedTarget(
        0,
        0,
        1,
        0,
        id,
        -1,
        0f,
        new Transform3d(new Translation3d(2, 0, 0), new Rotation3d()),
        new Transform3d(),
        ambiguity,
        corners,
        corners);
  }
}
