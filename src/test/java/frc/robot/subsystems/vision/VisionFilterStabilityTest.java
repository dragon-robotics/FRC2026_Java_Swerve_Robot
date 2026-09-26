// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
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
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
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
 * pure-math WPILib {@link SwerveDrivePoseEstimator} (not the native CTRE estimator) with a
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
  void invalidNumbersAndUnsupportedTagMetadataCannotReachFusion() {
    Pose3d pose = new Pose3d(4, 4, 0, new Rotation3d());
    for (PoseObservation observation :
        List.of(
            new PoseObservation(
                1, pose, .1, 2, Double.NaN, PoseObservationType.PHOTONVISION, new int[] {2, 3}),
            new PoseObservation(
                1, pose, .1, 2, 2, PoseObservationType.PHOTONVISION, new int[] {2, 2}),
            new PoseObservation(
                1, pose, .1, 2, 2, PoseObservationType.PHOTONVISION, new int[] {2, 99}),
            new PoseObservation(
                1, pose, -1, 1, 2, PoseObservationType.PHOTONVISION, new int[] {2}))) {
      assertTrue(VisionSubsystem.rejectionReason(observation).isPresent());
    }
  }

  @Test
  void goodMultiTagObservationsAreAccepted() {
    assertTrue(
        VisionSubsystem.rejectionReason(VisionScenarios.goodMultiTag(4.0, 4.0, 0.0, 0.0)).isEmpty(),
        "Good multi-tag pose should be accepted");
  }

  @Test
  void multitagInitializationHandlesOutOfOrderCrossCameraTimestamps() throws Exception {
    VisionSubsystem vision = new VisionSubsystem(null, (pose, timestamp, stdDevs) -> {});
    Method trackMultitagInitialization =
        VisionSubsystem.class.getDeclaredMethod(
            "trackMultitagInitialization", PoseObservation.class, Pose2d.class, String.class);
    trackMultitagInitialization.setAccessible(true);
    Field initializationComplete =
        VisionSubsystem.class.getDeclaredField("visionInitializationComplete");
    initializationComplete.setAccessible(true);

    assertFalse(
        initializationComplete.getBoolean(vision), "Vision initialization should start incomplete");

    for (int i = 0; i < VisionSubsystem.requiredStableMultitagPosesForInitialization(); i++) {
      PoseObservation newerFrontCameraObservation =
          multitagCoprocessorObs(4.0 + (0.01 * i), 4.0, 0.0, 10.0 + (0.02 * i));
      PoseObservation olderRearCameraObservation =
          multitagCoprocessorObs(4.0 + (0.01 * i), 4.0, 0.0, 9.0 + (0.02 * i));

      trackMultitagInitialization.invoke(
          vision,
          newerFrontCameraObservation,
          newerFrontCameraObservation.pose().toPose2d(),
          "front");
      trackMultitagInitialization.invoke(
          vision, olderRearCameraObservation, olderRearCameraObservation.pose().toPose2d(), "rear");
    }

    assertTrue(
        initializationComplete.getBoolean(vision),
        "Stable MultiTag streak from one camera should survive older frames from another camera");
  }

  @Test
  void multitagInitializationIgnoresNonCandidateObservations() throws Exception {
    VisionSubsystem vision = new VisionSubsystem(null, (pose, timestamp, stdDevs) -> {});
    Method trackMultitagInitialization =
        VisionSubsystem.class.getDeclaredMethod(
            "trackMultitagInitialization", PoseObservation.class, Pose2d.class, String.class);
    trackMultitagInitialization.setAccessible(true);
    Field initializationComplete =
        VisionSubsystem.class.getDeclaredField("visionInitializationComplete");
    initializationComplete.setAccessible(true);

    for (int i = 0; i < VisionSubsystem.requiredStableMultitagPosesForInitialization(); i++) {
      PoseObservation multitagObservation =
          multitagCoprocessorObs(4.0 + (0.01 * i), 4.0, 0.0, 20.0 + (0.02 * i));
      PoseObservation singleTagObservation =
          singleTagPhotonVisionObs(4.0 + (0.01 * i), 4.0, 0.0, 20.01 + (0.02 * i));

      trackMultitagInitialization.invoke(
          vision, multitagObservation, multitagObservation.pose().toPose2d(), "front");
      trackMultitagInitialization.invoke(
          vision, singleTagObservation, singleTagObservation.pose().toPose2d(), "front");
    }

    assertTrue(
        initializationComplete.getBoolean(vision),
        "Single-tag/fallback observations should not reset stable MultiTag initialization");
  }

  @Test
  void uncertaintyUsesDistanceAndRealTagCountWithoutAimingBoost() {
    Pose3d pose = new Pose3d(4.0, 4.0, 0.0, new Rotation3d());
    PoseObservation single =
        new PoseObservation(0, pose, .1, 1, 2, PoseObservationType.PHOTONVISION, new int[] {2});
    PoseObservation multi =
        new PoseObservation(
            0,
            pose,
            .1,
            2,
            2,
            PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            new int[] {2, 3});
    PoseObservation far =
        new PoseObservation(
            0,
            pose,
            .1,
            2,
            4,
            PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            new int[] {2, 3});
    double multiSigma = VisionSubsystem.standardDeviations(multi, 0, false).get(0, 0);
    assertTrue(multiSigma > 0);
    assertEquals(
        10 * multiSigma, VisionSubsystem.standardDeviations(single, 0, false).get(0, 0), 1e-9);
    assertEquals(4 * multiSigma, VisionSubsystem.standardDeviations(far, 0, false).get(0, 0), 1e-9);
    assertEquals(multiSigma, VisionSubsystem.standardDeviations(multi, 0, true).get(0, 0), 1e-9);
    assertEquals(1e9, VisionSubsystem.standardDeviations(multi, 0, false).get(2, 0), 1e-3);
  }

  @Test
  void visionInputsCopyFromDefensivelyCopiesMutableArrays() {
    VisionIOInputs source = new VisionIOInputs();
    PoseObservation originalObservation = singleTagPhotonVisionObs(4.0, 4.0, 0.0, 1.0);
    PoseObservation replacementObservation = singleTagPhotonVisionObs(5.0, 5.0, 0.0, 2.0);
    source.setPoseObservations(new PoseObservation[] {originalObservation});
    source.setTagIds(new int[] {2, 3});

    VisionIOInputs copied = new VisionIOInputs();
    copied.copyFrom(source);

    PoseObservation[] sourceObservations = source.getPoseObservations();
    int[] sourceTagIds = source.getTagIds();
    sourceObservations[0] = replacementObservation;
    sourceTagIds[0] = 99;

    assertNotSame(sourceObservations, copied.getPoseObservations());
    assertNotSame(sourceTagIds, copied.getTagIds());
    assertEquals(originalObservation, copied.getPoseObservations()[0]);
    assertArrayEquals(new int[] {2, 3}, copied.getTagIds());
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

  private static PoseObservation multitagCoprocessorObs(
      double x, double y, double headingRad, double timestamp) {
    return new PoseObservation(
        timestamp,
        new Pose3d(x, y, 0.0, new Rotation3d(0.0, 0.0, headingRad)),
        0.05,
        2,
        2.0,
        PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
        new int[] {2, 3});
  }

  private static PoseObservation singleTagPhotonVisionObs(
      double x, double y, double headingRad, double timestamp) {
    return new PoseObservation(
        timestamp,
        new Pose3d(x, y, 0.0, new Rotation3d(0.0, 0.0, headingRad)),
        0.05,
        1,
        2.0,
        PoseObservationType.PHOTONVISION,
        new int[] {2});
  }
}
