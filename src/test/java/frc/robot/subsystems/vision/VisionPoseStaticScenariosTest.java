// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
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
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/**
 * Regression suite for vision pose stability at two known-problematic field positions (left side
 * Y=7.279 m and right side Y=0.650 m), tested at the four yaw angles that directly face each camera
 * into the scoring structure.
 *
 * <p>These near-edge viewpoints exercise camera noise and pose ambiguity. The production camera
 * configuration currently includes front, right and left cameras; the two back-facing scenarios
 * also check behavior without their intended camera and do not require vision coverage.
 *
 * <p><b>What this test checks:</b>
 *
 * <ul>
 *   <li>Max single-cycle odometry jump during the 5-second measurement window must stay below
 *       {@link #MAX_JUMP_M} (stationary robot should not jump at all).
 *   <li>Max deviation of any accepted vision pose from the known ground-truth position must stay
 *       below {@link #MAX_VISION_DEVIATION_M}; estimator weighting cannot improve raw solver
 *       accuracy.
 * </ul>
 *
 * <p>The PhotonVision sim receives each scenario's ground-truth pose directly, so accepted vision
 * poses are checked against an independent reference instead of the drivetrain estimator pose they
 * are meant to validate.
 *
 * <p>Tagged {@code sim}; required HAL and container initialization failures fail the test.
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

  /** Maximum permitted native estimator correction per cycle for this stationary fixture. */
  private static final double MAX_JUMP_M = 0.15;

  private static final double MAX_HEADING_DEVIATION_DEGREES = 2.0;

  /** Broad raw-pose safety bound; detailed solver RMSE and outliers are checked in replay. */
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
      Path logDirectory = Path.of("build", "vision-stability", "logs").toAbsolutePath();
      Files.createDirectories(logDirectory);
      DataLogManager.start(
          logDirectory.toString(), "vision-static-" + System.currentTimeMillis() + ".wpilog");
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setAutonomous(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      container = new RobotContainer();
      // A canceled default command reschedules and retains its old heading target across poses.
      // Remove it so the native drivetrain stays stationary, just like the camera ground truth.
      container.swerveSubsystem.removeDefaultCommand();
    } catch (Throwable t) {
      throw new AssertionError("Static scenario initialization failed", t);
    }
  }

  @AfterAll
  static void tearDownHal() {
    try {
      CommandScheduler.getInstance().cancelAll();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      DataLogManager.getLog().flush();
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
    assertTrue(halReady, "HAL/simulation unavailable in this environment");
    assertTrue(container != null, "RobotContainer failed to initialize");

    // Cancel any residual commands; reset drivetrain pose to scenario start.
    CommandScheduler.getInstance().cancelAll();
    container.swerveSubsystem.setControl(
        new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds()));
    container.swerveSubsystem.resetPose(s.pose());
    container.setVisionSimulationPoseSupplier(s::pose);

    List<String> csv = new ArrayList<>();
    csv.add(
        "cycle,phase,t_s,"
            + "odomX,odomY,odomYawDeg,"
            + "visionX,visionY,visionYawDeg,"
            + "odomJump_m,visionDeviation_m,hasVision,odomHeadingDeviation_deg");

    Pose2d prev = s.pose();
    double maxOdomJump = 0.0;
    double maxFusedError = 0.0;
    double lastSnapshotTimestamp = Double.NEGATIVE_INFINITY;
    double maxHeadingDeviationDegrees = 0.0;
    double maxVisionDeviation = 0.0;
    double sumVisionDeviation = 0.0;
    int visionCycles = 0;
    int totalCycles = WARMUP_CYCLES + MEASURE_CYCLES;

    for (int cycle = 0; cycle < totalCycles; cycle++) {
      // Emulate normal DS packets; DataLogManager pauses after ten seconds without refreshes.
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(DT);

      Pose2d odom = container.swerveSubsystem.getState().Pose;
      Optional<VisionSubsystem.AcceptedObservationSnapshot> vis =
          container.visionSubsystem.getLatestAcceptedObservationSnapshot();

      double odomJump = odom.getTranslation().getDistance(prev.getTranslation());
      double headingDeviationDegrees =
          Math.abs(odom.getRotation().minus(s.pose().getRotation()).getDegrees());
      boolean measuring = cycle >= WARMUP_CYCLES;

      double visionDev = Double.NaN;
      if (vis.isPresent()) {
        visionDev = vis.get().pose().getTranslation().getDistance(s.pose().getTranslation());
        if (measuring && vis.get().timestamp() > lastSnapshotTimestamp) {
          visionCycles++;
          sumVisionDeviation += visionDev;
          maxVisionDeviation = Math.max(maxVisionDeviation, visionDev);
        }
      }
      if (vis.isPresent()) lastSnapshotTimestamp = vis.get().timestamp();
      if (measuring) {
        maxFusedError =
            Math.max(maxFusedError, odom.getTranslation().getDistance(s.pose().getTranslation()));
        maxOdomJump = Math.max(maxOdomJump, odomJump);
        maxHeadingDeviationDegrees = Math.max(maxHeadingDeviationDegrees, headingDeviationDegrees);
      }
      prev = odom;

      double t = cycle * DT;
      csv.add(
          String.format(
              "%d,%s,%.3f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%b,%.4f",
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
              vis.isPresent(),
              headingDeviationDegrees));
    }

    writeCsv("static-" + s.name() + ".csv", csv);

    double meanVisionDev = visionCycles > 0 ? sumVisionDeviation / visionCycles : 0.0;
    System.out.printf(
        "[VisionStatic|%-12s] groundTruth=(%.3f,%.3f,%.0f°)"
            + "  maxOdomJump=%.4f m  newestSnapshots=%3d/%d"
            + "  maxVisDev=%.4f m  meanVisDev=%.4f m  maxHeadingDev=%.4f deg%n",
        s.name(),
        s.x(),
        s.y(),
        s.yawDeg(),
        maxOdomJump,
        visionCycles,
        MEASURE_CYCLES,
        maxVisionDeviation,
        meanVisionDev,
        maxHeadingDeviationDegrees);

    // ── Assertions ───────────────────────────────────────────────────

    assertTrue(
        maxFusedError <= .35,
        "Fused pose error exceeded 0.35 m: " + s.name() + " " + maxFusedError);
    final double finalMaxOdomJump = maxOdomJump;
    final double finalMaxVisionDeviation = maxVisionDeviation;
    final double finalMaxHeadingDeviation = maxHeadingDeviationDegrees;

    assertTrue(
        finalMaxHeadingDeviation <= MAX_HEADING_DEVIATION_DEGREES,
        () ->
            String.format(
                "[%s] Stationary drivetrain heading deviated %.4f degrees from camera ground truth",
                s.name(), finalMaxHeadingDeviation));

    // B is intentionally absent from the production simulation camera set. Its two orientations
    // may have no usable tags; the six orientations aimed through active cameras must see vision.
    if (s != LEFT_BACK && s != RIGHT_BACK) {
      assertTrue(visionCycles > 0, "Active-camera scenario must exercise vision: " + s.name());
    }

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
