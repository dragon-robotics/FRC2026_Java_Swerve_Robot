// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.constants.VisionConstants;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import org.junit.jupiter.api.Test;

/**
 * HAL-free regression guard for the lean vision filter.
 *
 * <p>This test does not run the real drivetrain or Phoenix6/PhotonVision sim. Instead it drives a
 * pure-math {@link SwerveDrivePoseEstimator} (identical to the one inside the drivetrain) with a
 * stationary ground-truth robot, then injects synthetic vision observations through the <b>real</b>
 * production filter ({@link VisionSubsystem#rejectionReason} + {@link
 * VisionSubsystem#standardDeviations}). It asserts that the fused pose never teleports and stays
 * close to ground truth, even when realistic flip-vulnerable and adversarial observations are mixed
 * in.
 *
 * <p>Why this matters: the rewritten filter relies on std-dev weighting (not hard rejection) to
 * tame bad single-tag/flipped poses. This test pins that behavior so a future change that, say,
 * drops the distance scaling or slashes the std-dev baseline will fail loudly instead of silently
 * re-introducing the teleporting bug.
 */
class VisionFilterStabilityTest {

  private static final double DT = 0.02; // 50 Hz
  private static final int CYCLES = 250; // 5 seconds
  private static final double MAX_SINGLE_CYCLE_JUMP_M = 0.5;
  private static final double MAX_MEAN_DISCREPANCY_M = 0.3;

  /** Stationary ground-truth robot pose near the middle of a half-field. */
  private static final Pose2d GROUND_TRUTH = new Pose2d(4.0, 4.0, new Rotation2d());

  @Test
  void fusedPoseStaysStableUnderRealisticVisionNoise() throws IOException {
    Random rng = new Random(42); // deterministic
    Rotation2d gyro = new Rotation2d();
    SwerveModulePosition[] modules = zeroedModules();
    SwerveDrivePoseEstimator estimator =
        new SwerveDrivePoseEstimator(dummyKinematics(), gyro, modules, GROUND_TRUTH);

    List<String> csv = new ArrayList<>();
    csv.add("cycle,t,fusedX,fusedY,jump,injected,acceptedThisCycle,lastReason");

    Pose2d prev = estimator.getEstimatedPosition();
    double maxJump = 0.0;
    double sumDiscrepancy = 0.0;
    int acceptedCount = 0;
    int rejectedCount = 0;

    for (int cycle = 0; cycle < CYCLES; cycle++) {
      double t = cycle * DT;

      // Stationary robot: wheels don't move, gyro fixed. Only vision perturbs the
      // estimate.
      estimator.updateWithTime(t, gyro, modules);

      List<PoseObservation> stream = new ArrayList<>();
      // Two good multi-tag observations every cycle (the dominant, trustworthy
      // signal).
      stream.add(
          VisionScenarios.goodMultiTag(
              noisy(rng, GROUND_TRUTH.getX()), noisy(rng, GROUND_TRUTH.getY()), 0.0, t));
      stream.add(
          VisionScenarios.goodMultiTag(
              noisy(rng, GROUND_TRUTH.getX()), noisy(rng, GROUND_TRUTH.getY()), 0.0, t));

      String injected = "good";
      // Periodically inject a realistic flip-vulnerable single-tag pose ~4 m
      // off-truth.
      if (cycle % 25 == 0) {
        stream.add(
            VisionScenarios.flippedSingleTag(
                GROUND_TRUTH.getX() + 3.0, GROUND_TRUTH.getY() + 3.0, Math.PI, t));
        injected = "flipped";
      }
      // Periodically inject clearly-bad observations the gates must reject.
      if (cycle % 17 == 0) {
        stream.add(VisionScenarios.outOfBounds(t));
        stream.add(VisionScenarios.highZ(GROUND_TRUTH.getX(), GROUND_TRUTH.getY(), t));
        stream.add(VisionScenarios.tooFar(GROUND_TRUTH.getX(), GROUND_TRUTH.getY(), t));
        stream.add(
            VisionScenarios.singleTagHighAmbiguity(GROUND_TRUTH.getX(), GROUND_TRUTH.getY(), t));
      }

      String lastReason = "-";
      boolean acceptedThisCycle = false;
      for (PoseObservation obs : stream) {
        Optional<String> reason = VisionSubsystem.rejectionReason(obs);
        if (reason.isPresent()) {
          rejectedCount++;
          lastReason = reason.get();
          continue;
        }
        acceptedThisCycle = true;
        acceptedCount++;
        estimator.addVisionMeasurement(
            obs.pose().toPose2d(),
            obs.timestamp(),
            VisionSubsystem.standardDeviations(obs, 0, false));
      }

      Pose2d fused = estimator.getEstimatedPosition();
      double jump = fused.getTranslation().getDistance(prev.getTranslation());
      maxJump = Math.max(maxJump, jump);
      sumDiscrepancy += fused.getTranslation().getDistance(GROUND_TRUTH.getTranslation());
      prev = fused;

      csv.add(
          String.format(
              "%d,%.3f,%.4f,%.4f,%.4f,%s,%b,%s",
              cycle, t, fused.getX(), fused.getY(), jump, injected, acceptedThisCycle, lastReason));
    }

    double meanDiscrepancy = sumDiscrepancy / CYCLES;
    writeCsv("filter-test.csv", csv);

    final double observedMaxJump = maxJump;
    final double observedMeanDiscrepancy = meanDiscrepancy;
    assertTrue(acceptedCount > 0, "Expected some good observations to be accepted");
    assertTrue(rejectedCount > 0, "Expected adversarial observations to be rejected by the gates");
    assertTrue(
        observedMaxJump <= MAX_SINGLE_CYCLE_JUMP_M,
        () ->
            "Max single-cycle pose jump "
                + observedMaxJump
                + " m exceeded "
                + MAX_SINGLE_CYCLE_JUMP_M
                + " m — fused pose teleported. See build/vision-stability/filter-test.csv");
    assertTrue(
        observedMeanDiscrepancy <= MAX_MEAN_DISCREPANCY_M,
        () ->
            "Mean discrepancy "
                + observedMeanDiscrepancy
                + " m exceeded "
                + MAX_MEAN_DISCREPANCY_M
                + " m — fused pose drifted from ground truth");
  }

  @Test
  void adversarialObservationsAreRejected() {
    assertTrue(
        VisionSubsystem.rejectionReason(VisionScenarios.outOfBounds(0.0)).isPresent(),
        "Out-of-bounds pose should be rejected");
    assertTrue(
        VisionSubsystem.rejectionReason(VisionScenarios.highZ(4.0, 4.0, 0.0)).isPresent(),
        "High-Z pose should be rejected");
    assertTrue(
        VisionSubsystem.rejectionReason(VisionScenarios.tooFar(4.0, 4.0, 0.0)).isPresent(),
        "Too-far pose should be rejected");
    assertTrue(
        VisionSubsystem.rejectionReason(VisionScenarios.singleTagHighAmbiguity(4.0, 4.0, 0.0))
            .isPresent(),
        "High-ambiguity single-tag pose should be rejected");
  }

  @Test
  void goodMultiTagObservationsAreAccepted() {
    assertTrue(
        VisionSubsystem.rejectionReason(VisionScenarios.goodMultiTag(4.0, 4.0, 0.0, 0.0)).isEmpty(),
        "Good multi-tag pose should be accepted");
  }

  /**
   * Pins the single-tag translation distrust multiplier. A single tag gets {@code
   * SINGLE_TAG_LINEAR_STDDEV_MULTIPLIER} (5.0) applied to its translation std-dev, while multi-tag
   * observations do not. This is the fix that stops a flipped single-tag pose from teleporting the
   * fused estimate, so it is asserted explicitly.
   */
  @Test
  void singleTagTranslationStdDevAppliesTheFiveTimesMultiplier() {
    double distance = 2.0;
    Pose3d pose = new Pose3d(4.0, 4.0, 0.0, new Rotation3d());
    PoseObservation singleTag =
        new PoseObservation(
            0.0, pose, 0.1, 1, distance, PoseObservationType.PHOTONVISION, new int[] {1});
    PoseObservation twoTag =
        new PoseObservation(
            0.0, pose, 0.1, 2, distance, PoseObservationType.PHOTONVISION, new int[] {1, 2});

    double singleStdDev = VisionSubsystem.standardDeviations(singleTag, 0, false).get(0, 0);
    double twoStdDev = VisionSubsystem.standardDeviations(twoTag, 0, false).get(0, 0);

    // For a single tag: stdDev = LINEAR_STDDEV_BASELINE * dist^2 * 1(cam) * 1(aim) * multiplier.
    double singleTagBaseline = VisionConstants.LINEAR_STDDEV_BASELINE * distance * distance;
    double observedMultiplier = singleStdDev / singleTagBaseline;
    System.out.printf(
        "[StdDev] singleTag(%.0fm)=%.4f m, twoTag(%.0fm)=%.4f m, effective single-tag multiplier=%.2f%n",
        distance, singleStdDev, distance, twoStdDev, observedMultiplier);

    assertEquals(
        VisionConstants.SINGLE_TAG_LINEAR_STDDEV_MULTIPLIER,
        observedMultiplier,
        1e-9,
        "Single-tag translation std-dev must apply the configured 5x distrust multiplier");

    // Multi-tag must NOT receive the single-tag multiplier: stdDev = baseline * dist^2 / tagCount.
    double expectedTwoTag = VisionConstants.LINEAR_STDDEV_BASELINE * distance * distance / 2.0;
    assertEquals(
        expectedTwoTag,
        twoStdDev,
        1e-9,
        "Multi-tag std-dev must not apply the single-tag distrust multiplier");
  }

  private static SwerveDriveKinematics dummyKinematics() {
    double o = 0.3;
    return new SwerveDriveKinematics(
        new Translation2d(o, o),
        new Translation2d(o, -o),
        new Translation2d(-o, o),
        new Translation2d(-o, -o));
  }

  private static SwerveModulePosition[] zeroedModules() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
  }

  private static double noisy(Random rng, double value) {
    return value + (rng.nextDouble() - 0.5) * 0.04; // +/- 2 cm
  }

  private static void writeCsv(String name, List<String> lines) throws IOException {
    Path dir = Path.of("build", "vision-stability");
    Files.createDirectories(dir);
    Files.write(dir.resolve(name), lines);
  }
}
