// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/**
 * Regression suite for vision pose stability at two known-problematic field positions (left side
 * Y=7.279 m and right side Y=0.650 m), tested at the four yaw angles that directly face each camera
 * into the scoring structure.
 *
 * <p><b>Problem background:</b> at these near-edge positions the cameras look at the central
 * scoring Hub from the side. When a camera sees two or more tags on the <em>same</em> Hub face, the
 * multi-tag PnP geometry is coplanar (planar ambiguity identical to single-tag): the solver
 * produces two mirror solutions, occasionally picks the wrong one, and — because it is logged as a
 * multi-tag observation — the result bypasses the single-tag std-dev penalty and is accepted with
 * full confidence. This causes large, periodic odometry jumps.
 *
 * <p><b>What this test checks:</b>
 *
 * <ul>
 *   <li>Max single-cycle odometry jump during the 5-second measurement window must stay below
 *       {@link #MAX_JUMP_M} (stationary robot should not jump at all).
 *   <li>Max deviation of any accepted vision pose from the known ground-truth position must stay
 *       below {@link #MAX_VISION_DEVIATION_M} (bad poses must be rejected or de-weighted enough not
 *       to pull the estimator far from truth).
 * </ul>
 *
 * <p>The PhotonVision sim receives each scenario's ground-truth pose directly, so accepted vision
 * poses are checked against an independent reference instead of the drivetrain estimator pose they
 * are meant to validate.
 *
 * <p>Tagged {@code sim} and guarded with JUnit {@link Assumptions}: skipped rather than failed when
 * HAL/simulation is unavailable.
 *
 * <p>Run with: {@code ./gradlew visionStabilityTest}
 */
@Tag("sim")
class VisionPoseStaticScenariosTest {

  // ────────────────────────────────────────────────────────────────────
  // Scenario definitions
  // ────────────────────────────────────────────────────────────────────

  private record Scenario(String name, double x, double y, double yawDeg) {
    Pose2d pose() {
      return new Pose2d(x, y, Rotation2d.fromDegrees(yawDeg));
    }
  }

  /**
   * Left side (Y = 7.279 m). Yaws chosen so each camera faces the Hub in turn: front camera → −90°,
   * back → +90°, left (pointing +Y wall) → +180°, right → 0°.
   */
  private static final Scenario LEFT_FRONT = new Scenario("left_front", 4.407, 7.279, -90);

  private static final Scenario LEFT_BACK = new Scenario("left_back", 4.407, 7.279, 90);
  private static final Scenario LEFT_LEFT = new Scenario("left_left", 4.407, 7.279, 180);
  private static final Scenario LEFT_RIGHT = new Scenario("left_right", 4.407, 7.279, 0);

  /**
   * Right side (Y = 0.650 m). Mirror yaws: front camera → +90°, back → −90°, left → 0°, right →
   * +180°.
   */
  private static final Scenario RIGHT_FRONT = new Scenario("right_front", 4.407, 0.650, 90);

  private static final Scenario RIGHT_BACK = new Scenario("right_back", 4.407, 0.650, -90);
  private static final Scenario RIGHT_LEFT = new Scenario("right_left", 4.407, 0.650, 0);
  private static final Scenario RIGHT_RIGHT = new Scenario("right_right", 4.407, 0.650, 180);

  // ────────────────────────────────────────────────────────────────────
  // Thresholds
  // ────────────────────────────────────────────────────────────────────

  private static final double DT = 0.02; // 50 Hz

  /** Warmup cycles before recording metrics. Lets the validator build a baseline. */
  private static final int WARMUP_CYCLES = 100; // 2 s

  /** Measurement cycles after warmup. */
  private static final int MEASURE_CYCLES = 250; // 5 s

  /**
   * A stationary robot's odometry must not jump more than this per cycle. Any jump larger than ~2
   * cm is a sign that a bad vision pose was accepted and fused with high confidence.
   */
  private static final double MAX_JUMP_M = 0.15;

  /**
   * Maximum allowed distance between any accepted vision pose and the robot's known ground-truth
   * position. Poses farther away are flipped/wrong and must not reach the Kalman filter — or if
   * they do, the std-dev must be large enough that they don't move the estimate appreciably
   * (checked separately by MAX_JUMP_M).
   */
  private static final double MAX_VISION_DEVIATION_M = 1.5;

  // ────────────────────────────────────────────────────────────────────
  // HAL / shared container setup
  // ────────────────────────────────────────────────────────────────────

  private static boolean halReady = false;
  private static RobotContainer container;

  @BeforeAll
  static void setUpHal() {
    try {
      halReady = HAL.initialize(500, 0);
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setAutonomous(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      container = new RobotContainer();
    } catch (Throwable t) {
      halReady = false;
    }
  }

  @AfterAll
  static void tearDownHal() {
    try {
      CommandScheduler.getInstance().cancelAll();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    } catch (Throwable ignored) {
      // best effort
    }
    if (halReady) {
      HAL.shutdown();
    }
  }

  // ────────────────────────────────────────────────────────────────────
  // Individual test methods (one per scenario)
  // ────────────────────────────────────────────────────────────────────

  @Test
  void leftFront() throws IOException {
    runScenario(LEFT_FRONT);
  }

  @Test
  void leftBack() throws IOException {
    runScenario(LEFT_BACK);
  }

  @Test
  void leftLeft() throws IOException {
    runScenario(LEFT_LEFT);
  }

  @Test
  void leftRight() throws IOException {
    runScenario(LEFT_RIGHT);
  }

  @Test
  void rightFront() throws IOException {
    runScenario(RIGHT_FRONT);
  }

  @Test
  void rightBack() throws IOException {
    runScenario(RIGHT_BACK);
  }

  @Test
  void rightLeft() throws IOException {
    runScenario(RIGHT_LEFT);
  }

  @Test
  void rightRight() throws IOException {
    runScenario(RIGHT_RIGHT);
  }

  // ────────────────────────────────────────────────────────────────────
  // Core scenario runner
  // ────────────────────────────────────────────────────────────────────

  private void runScenario(Scenario s) throws IOException {
    Assumptions.assumeTrue(halReady, "HAL/simulation unavailable in this environment");
    Assumptions.assumeTrue(container != null, "RobotContainer failed to initialize");

    // Cancel any residual commands; reset drivetrain pose to scenario start.
    CommandScheduler.getInstance().cancelAll();
    container.swerveSubsystem.resetPose(s.pose());
    container.setVisionSimulationPoseSupplier(s::pose);

    List<String> csv = new ArrayList<>();
    csv.add(
        "cycle,phase,t_s,"
            + "odomX,odomY,odomYawDeg,"
            + "visionX,visionY,visionYawDeg,"
            + "odomJump_m,visionDeviation_m,hasVision");

    Pose2d prev = s.pose();
    double maxOdomJump = 0.0;
    double maxVisionDeviation = 0.0;
    double sumVisionDeviation = 0.0;
    int visionCycles = 0;
    int totalCycles = WARMUP_CYCLES + MEASURE_CYCLES;

    for (int cycle = 0; cycle < totalCycles; cycle++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(DT);

      Pose2d odom = container.swerveSubsystem.getState().Pose;
      Optional<VisionSubsystem.AcceptedObservationSnapshot> vis =
          container.visionSubsystem.getLatestAcceptedObservationSnapshot();

      double odomJump = odom.getTranslation().getDistance(prev.getTranslation());
      boolean measuring = cycle >= WARMUP_CYCLES;

      double visionDev = Double.NaN;
      if (vis.isPresent()) {
        visionDev = vis.get().pose().getTranslation().getDistance(s.pose().getTranslation());
        if (measuring) {
          visionCycles++;
          sumVisionDeviation += visionDev;
          maxVisionDeviation = Math.max(maxVisionDeviation, visionDev);
        }
      }
      if (measuring) {
        maxOdomJump = Math.max(maxOdomJump, odomJump);
      }
      prev = odom;

      double t = cycle * DT;
      csv.add(
          String.format(
              "%d,%s,%.3f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%b",
              cycle,
              measuring ? "measure" : "warmup",
              t,
              odom.getX(),
              odom.getY(),
              odom.getRotation().getDegrees(),
              vis.map(v -> v.pose().getX()).orElse(Double.NaN),
              vis.map(v -> v.pose().getY()).orElse(Double.NaN),
              vis.map(v -> v.pose().getRotation().getDegrees()).orElse(Double.NaN),
              odomJump,
              visionDev,
              vis.isPresent()));
    }

    writeCsv("static-" + s.name() + ".csv", csv);

    double meanVisionDev = visionCycles > 0 ? sumVisionDeviation / visionCycles : 0.0;
    System.out.printf(
        "[VisionStatic|%-12s] groundTruth=(%.3f,%.3f,%.0f°)"
            + "  maxOdomJump=%.4f m  visionCycles=%3d/%d"
            + "  maxVisDev=%.4f m  meanVisDev=%.4f m%n",
        s.name(),
        s.x(),
        s.y(),
        s.yawDeg(),
        maxOdomJump,
        visionCycles,
        MEASURE_CYCLES,
        maxVisionDeviation,
        meanVisionDev);

    // ── Assertions ───────────────────────────────────────────────────

    final double finalMaxOdomJump = maxOdomJump;
    final double finalMaxVisionDeviation = maxVisionDeviation;

    assertTrue(
        finalMaxOdomJump <= MAX_JUMP_M,
        () ->
            String.format(
                "[%s] Odometry jumped %.4f m in one cycle for a stationary robot "
                    + "(threshold %.2f m). A bad vision pose was fused with high confidence. "
                    + "See build/vision-stability/static-%s.csv",
                s.name(), finalMaxOdomJump, MAX_JUMP_M, s.name()));

    if (visionCycles > 0) {
      assertTrue(
          finalMaxVisionDeviation <= MAX_VISION_DEVIATION_M,
          () ->
              String.format(
                  "[%s] Accepted vision pose was %.4f m from ground truth "
                      + "(threshold %.2f m). A flipped/wrong pose passed the rejection gate. "
                      + "See build/vision-stability/static-%s.csv",
                  s.name(), finalMaxVisionDeviation, MAX_VISION_DEVIATION_M, s.name()));
    }
  }

  // ────────────────────────────────────────────────────────────────────
  // CSV helper
  // ────────────────────────────────────────────────────────────────────

  private static void writeCsv(String name, List<String> lines) throws IOException {
    Path dir = Path.of("build", "vision-stability");
    Files.createDirectories(dir);
    Files.write(dir.resolve(name), lines);
  }
}
