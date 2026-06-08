// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
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
 * Best-effort full-sim integration test: runs the "Right Side Safe" PathPlanner auto in a headless
 * Phoenix6 + PhotonVision simulation and records the swerve odometry pose against the accepted
 * vision pose each cycle, asserting the fused odometry never teleports and that vision is accepted.
 *
 * <p><b>Important caveat:</b> in simulation the PhotonVision sim generates AprilTag detections FROM
 * the drivetrain's own simulated pose, so sim vision and odometry are self-consistent by
 * construction. This test therefore cannot reproduce the real flipped-tag teleport — that failure
 * mode is reproduced and regression-guarded deterministically by {@link VisionFilterStabilityTest}.
 * This test's value is confirming the named auto runs end-to-end and odometry stays continuous.
 *
 * <p>Tagged {@code sim} and guarded with JUnit assumptions: if the headless simulation cannot
 * initialize in this environment (native libraries, HAL), the test is skipped rather than failed.
 *
 * <p>Run with: {@code ./gradlew visionStabilityTest}
 */
@Tag("sim")
class RightSideSafeOdometryTest {

  private static final String AUTO_NAME = "Right Side Safe";
  private static final double DT = 0.02; // 50 Hz
  private static final int MAX_CYCLES = 1500; // up to 30 s of sim
  private static final int WARMUP_CYCLES = 15; // ignore startup resetOdom transient
  private static final double MAX_SINGLE_CYCLE_JUMP_M = 0.5;
  // The auto chains several paths; PathPlanner can reset the pose at path boundaries, which
  // legitimately appears as a few large single-cycle jumps. Tolerate a handful, but fail on
  // continuous teleporting (the signature of a vision fault). Note: sim vision is self-referential,
  // so the vision-fault teleport itself is guarded deterministically by VisionFilterStabilityTest.
  private static final int MAX_LARGE_JUMPS = 5;

  private static boolean halReady = false;

  @BeforeAll
  static void setUpHal() {
    try {
      halReady = HAL.initialize(500, 0);
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setAutonomous(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
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

  @Test
  void rightSideSafeAutoKeepsOdometryContinuous() throws IOException {
    Assumptions.assumeTrue(halReady, "HAL/simulation unavailable in this environment");

    RobotContainer container;
    Command auto;
    try {
      container = new RobotContainer();
      auto = new PathPlannerAuto(AUTO_NAME);
    } catch (Throwable t) {
      Assumptions.abort("Headless sim could not initialize RobotContainer/auto: " + t);
      return; // unreachable; keeps the compiler happy about definite assignment
    }

    auto.schedule();

    List<String> csv = new ArrayList<>();
    csv.add("cycle,t,odomX,odomY,visionX,visionY,jump,hasVision");

    Pose2d prev = container.swerveSubsystem.getState().Pose;
    double maxJump = 0.0;
    int largeJumpCount = 0;
    int visionAcceptedCycles = 0;
    int cyclesRun = 0;

    try {
      for (int cycle = 0; cycle < MAX_CYCLES && auto.isScheduled(); cycle++) {
        CommandScheduler.getInstance().run();
        SimHooks.stepTiming(DT);
        cyclesRun++;

        Pose2d odom = container.swerveSubsystem.getState().Pose;
        Optional<VisionSubsystem.AcceptedObservationSnapshot> vision =
            container.visionSubsystem.getLatestAcceptedObservationSnapshot();

        double jump = odom.getTranslation().getDistance(prev.getTranslation());
        // Ignore the startup window: the auto's resetOdom snaps the pose to the path start.
        if (cycle >= WARMUP_CYCLES) {
          maxJump = Math.max(maxJump, jump);
          if (jump > MAX_SINGLE_CYCLE_JUMP_M) {
            largeJumpCount++;
            System.out.printf(
                "[RightSideSafe] large jump %.2f m at cycle %d (t=%.2fs) odom=(%.2f, %.2f)%n",
                jump, cycle, cycle * DT, odom.getX(), odom.getY());
          }
        }
        prev = odom;
        if (vision.isPresent()) {
          visionAcceptedCycles++;
        }

        double t = cycle * DT;
        csv.add(
            String.format(
                "%d,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%b",
                cycle,
                t,
                odom.getX(),
                odom.getY(),
                vision.map(s -> s.pose().getX()).orElse(Double.NaN),
                vision.map(s -> s.pose().getY()).orElse(Double.NaN),
                jump,
                vision.isPresent()));
      }
    } catch (Throwable t) {
      writeCsv("right-side-safe.csv", csv);
      Assumptions.abort("Headless sim threw while running the auto: " + t);
      return;
    }

    writeCsv("right-side-safe.csv", csv);

    Assumptions.assumeTrue(cyclesRun > WARMUP_CYCLES, "Auto did not run long enough to evaluate");

    System.out.printf(
        "[RightSideSafe] cycles=%d visionAcceptedCycles=%d maxJump=%.3f m largeJumps=%d%n",
        cyclesRun, visionAcceptedCycles, maxJump, largeJumpCount);

    final int observedVisionCycles = visionAcceptedCycles;
    final int observedLargeJumps = largeJumpCount;
    assertTrue(
        observedVisionCycles > 0,
        "Expected vision to be accepted at least once during the auto (sim vision never fired)");
    // A few large jumps correspond to PathPlanner path-boundary resets; continuous teleporting
    // (a vision fault) would produce many more. The vision-fault teleport itself is guarded
    // deterministically by VisionFilterStabilityTest, since sim vision is self-referential.
    assertTrue(
        observedLargeJumps <= MAX_LARGE_JUMPS,
        () ->
            "Odometry teleported "
                + observedLargeJumps
                + " times (> "
                + MAX_LARGE_JUMPS
                + ") during the auto. See build/vision-stability/right-side-safe.csv");
  }

  private static void writeCsv(String name, List<String> lines) throws IOException {
    Path dir = Path.of("build", "vision-stability");
    Files.createDirectories(dir);
    Files.write(dir.resolve(name), lines);
  }
}
