package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.nio.file.*;
import java.util.*;
import org.junit.jupiter.api.Test;

/** Independent moving truth with delayed, faulty camera observations; no renderer or hardware. */
class VisionConsensusExperimentTest {
  @Test
  void captureTimeAlignmentPreservesOriginalMeasurementAndIgnoresCameraYaw() {
    var early = candidate(0, 1.0, 4.0, 3.9, Math.PI);
    var late = candidate(1, 1.15, 4.6, 4.5, 0.0);
    var decision = VisionSubsystem.evaluateConsensus(List.of(early, late), false);
    assertEquals(2, decision.support());
    assertTrue(decision.selected().isPresent(), "0.6 m real motion is not camera disagreement");
    var selected = decision.selected().get();
    assertTrue(selected == early || selected == late);
    assertEquals(selected == early ? 1.0 : 1.15, selected.observation().timestamp(), 1e-9);
    assertEquals(selected == early ? 4.0 : 4.6, selected.visionPose().getX(), 1e-9);
  }

  @Test
  void largeSoloCorrectionNeedsCorroborationButDisabledCanLocalize() {
    var wrong = candidate(0, 1.0, 5.5, 4.0, 0.0);
    assertEquals(
        "LARGE_SINGLE_CAMERA_DELTA",
        VisionSubsystem.evaluateConsensus(List.of(wrong), false).reason());
    assertTrue(VisionSubsystem.evaluateConsensus(List.of(wrong), true).selected().isPresent());
    var corroborating = candidate(1, 1.0, 5.55, 4.0, 0.0);
    assertEquals(
        2, VisionSubsystem.evaluateConsensus(List.of(wrong, corroborating), false).support());
    var small = candidate(0, 1.0, 4.1, 4.0, 0.0);
    assertTrue(VisionSubsystem.evaluateConsensus(List.of(small), false).selected().isPresent());
    assertEquals(0.1, VisionSubsystem.fusionStandardDeviations(small, 1).get(0, 0), 1e-9);
    assertEquals(1e9, VisionSubsystem.fusionStandardDeviations(small, 1).get(2, 0));
  }

  @Test
  void movingEstimatorRejectsFaultsWithoutStarvingHealthyVision() throws Exception {
    var kinematics =
        new SwerveDriveKinematics(
            new Translation2d(.3, .3),
            new Translation2d(.3, -.3),
            new Translation2d(-.3, .3),
            new Translation2d(-.3, -.3));
    var estimator =
        new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), modules(0), truth(1.0));
    List<String> csv = new ArrayList<>();
    csv.add("cycle,phase,t,truthX,fusedX,error,excessJump,decision,support,selectedAge,computeNs");
    List<Long> timings = new ArrayList<>();
    int accepted = 0;
    int faultRejections = 0;
    double maxError = 0;
    double maxExcessJump = 0;
    Pose2d previous = truth(1.0);
    for (int cycle = 0; cycle < 300; cycle++) {
      double t = 1.0 + cycle * .02;
      estimator.updateWithTime(t, new Rotation2d(), modules(2.0 * (t - 1.0) * 1.01));
      if (cycle < 10) {
        previous = estimator.getEstimatedPosition();
        continue;
      }
      String phase =
          cycle < 60
              ? "healthy"
              : cycle < 90
                  ? "backlog"
                  : cycle < 120
                      ? "split"
                      : cycle < 150
                          ? "solo_good"
                          : cycle < 180 ? "solo_bad" : cycle < 210 ? "bridge" : "recovery";
      int cameraCount = phase.startsWith("solo") ? 1 : phase.equals("bridge") ? 3 : 4;
      List<VisionSubsystem.ConsensusCandidate> candidates = new ArrayList<>();
      for (int camera = 0; camera < cameraCount; camera++) {
        double capture = t - (.02 + camera * .04);
        double offset =
            phase.equals("backlog") && camera == 0
                ? 1.5
                : phase.equals("split") && camera >= 2
                    ? 1.5
                    : phase.equals("solo_bad") ? 1.5 : phase.equals("bridge") ? camera * .4 : 0.0;
        Pose2d reference = estimator.sampleAt(capture).orElseThrow();
        double x = truth(capture).getX() + offset + .005 * Math.sin(cycle + camera);
        var observation = candidate(camera, capture, x, reference.getX(), 0.0);
        candidates.add(observation);
        if (phase.equals("backlog") && camera == 0) {
          for (int i = 0; i < 8; i++)
            candidates.add(candidate(camera, capture - i * .001, x, reference.getX(), 0.0));
        }
      }
      long start = System.nanoTime();
      var decision = VisionSubsystem.evaluateConsensus(candidates, false);
      long elapsed = System.nanoTime() - start;
      timings.add(elapsed);
      boolean rejectPhase =
          phase.equals("split") || phase.equals("solo_bad") || phase.equals("bridge");
      assertEquals(!rejectPhase, decision.selected().isPresent(), "cycle " + cycle + " " + phase);
      double age = -1;
      if (decision.selected().isPresent()) {
        var chosen = decision.selected().get();
        age = t - chosen.observation().timestamp();
        estimator.addVisionMeasurement(
            chosen.visionPose(),
            chosen.observation().timestamp(),
            VisionSubsystem.fusionStandardDeviations(chosen, decision.support()));
        accepted++;
      } else {
        faultRejections++;
        assertEquals(
            phase.equals("solo_bad") ? "LARGE_SINGLE_CAMERA_DELTA" : "CAMERA_DISAGREEMENT",
            decision.reason());
      }
      Pose2d fused = estimator.getEstimatedPosition();
      double error = fused.getTranslation().getDistance(truth(t).getTranslation());
      double excess =
          Math.max(0, fused.getTranslation().getDistance(previous.getTranslation()) - .04);
      maxError = Math.max(maxError, error);
      maxExcessJump = Math.max(maxExcessJump, excess);
      previous = fused;
      csv.add(
          String.format(
              Locale.US,
              "%d,%s,%.3f,%.6f,%.6f,%.6f,%.6f,%s,%d,%.3f,%d",
              cycle,
              phase,
              t,
              truth(t).getX(),
              fused.getX(),
              error,
              excess,
              decision.reason(),
              decision.support(),
              age,
              elapsed));
    }
    assertEquals(200, accepted);
    assertEquals(90, faultRejections);
    assertTrue(maxError < .15, "Moving truth error " + maxError);
    assertTrue(maxExcessJump < .1, "Excess jump " + maxExcessJump);
    Collections.sort(timings);
    Path dir = Path.of("build", "vision-experiment");
    Files.createDirectories(dir);
    Files.write(dir.resolve("moving-fault-injection.csv"), csv);
    String summary =
        String.format(
            Locale.US,
            "accepted=%d rejected=%d maxError=%.6f m maxExcessJump=%.6f m selector p50=%.3f us p95=%.3f us max=%.3f us",
            accepted,
            faultRejections,
            maxError,
            maxExcessJump,
            timings.get(timings.size() / 2) / 1e3,
            timings.get((int) (timings.size() * .95)) / 1e3,
            timings.get(timings.size() - 1) / 1e3);
    Files.writeString(dir.resolve("moving-fault-injection-summary.txt"), summary);
    System.out.println("[ConsensusExperiment] " + summary);
  }

  private static Pose2d truth(double t) {
    return new Pose2d(2.0 + 2.0 * (t - 1.0), 4.0, new Rotation2d());
  }

  private static SwerveModulePosition[] modules(double distance) {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(distance, new Rotation2d()),
      new SwerveModulePosition(distance, new Rotation2d()),
      new SwerveModulePosition(distance, new Rotation2d()),
      new SwerveModulePosition(distance, new Rotation2d())
    };
  }

  private static VisionSubsystem.ConsensusCandidate candidate(
      int camera, double t, double x, double odomX, double yaw) {
    var pose = new Pose2d(x, 4, new Rotation2d(yaw));
    var observation =
        new PoseObservation(
            t,
            new Pose3d(pose),
            .05,
            2,
            2.0,
            PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            new int[] {2, 3});
    return new VisionSubsystem.ConsensusCandidate(
        camera,
        "camera" + camera,
        "Vision/camera" + camera,
        observation,
        pose,
        VecBuilder.fill(.05, .05, 1e9),
        Math.abs(x - odomX),
        new Translation2d(x - odomX, 0));
  }
}
