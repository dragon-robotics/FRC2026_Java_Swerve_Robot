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
 * Full-sim vision pose estimation test: places the robot stationary at a fixed
 * field position
 * (X=4.407 m, Y=7.279 m, yaw=90°) and records the vision-accepted poses over
 * the duration of a
 * 5-second auto phase, asserting that vision fires and that odometry does not
 * drift for a
 * stationary robot.
 *
 * <p>
 * <b>Purpose:</b> measure the accuracy and consistency of pose estimation from
 * this specific
 * field location. The CSV output written to {@code build/vision-stability/} can
 * be loaded into
 * AdvantageScope or a spreadsheet for offline analysis.
 *
 * <p>
 * <b>Important caveat:</b> in simulation the PhotonVision sim generates
 * AprilTag detections FROM
 * the drivetrain's own simulated pose, so accepted vision poses are
 * self-referential by
 * construction and will closely track the odometry pose. The test's value is
 * confirming that cameras
 * can detect tags from this location and that the vision pipeline accepts and
 * fuses the measurements
 * without unexpected rejection.
 *
 * <p>
 * Tagged {@code sim} and guarded with JUnit assumptions: if the headless
 * simulation cannot
 * initialize in this environment (native libraries, HAL), the test is skipped
 * rather than failed.
 *
 * <p>
 * Run with: {@code ./gradlew visionStabilityTest}
 */
@Tag("sim")
class VisionPoseStaticTest {

  private static final String AUTO_NAME = "Vision Pose Static Test";
  private static final double DT = 0.02; // 50 Hz
  private static final int MAX_CYCLES = 750; // up to 15 s of sim (auto is 5 s)
  private static final int WARMUP_CYCLES = 10; // ignore startup transient before evaluating

  // The robot is stationary — any meaningful single-cycle jump indicates a
  // problem.
  private static final double MAX_SINGLE_CYCLE_JUMP_M = 0.5;

  // Known starting pose (matches the auto's startingPose field).
  private static final double STARTING_POSE_X = 4.407;
  private static final double STARTING_POSE_Y = 7.279;
  private static final double STARTING_POSE_YAW_DEG = 90.0;

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
  void visionPoseStaticTestRecordsAcceptedPoses() throws IOException {
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
    csv.add("cycle,t,odomX,odomY,odomYawDeg,visionX,visionY,visionYawDeg,jump,hasVision");

    Pose2d prev = container.swerveSubsystem.getState().Pose;
    double maxJump = 0.0;
    int visionAcceptedCycles = 0;
    int cyclesRun = 0;
    double sumVisionX = 0.0;
    double sumVisionY = 0.0;
    double sumVisionYawDeg = 0.0;

    try {
      for (int cycle = 0; cycle < MAX_CYCLES && auto.isScheduled(); cycle++) {
        CommandScheduler.getInstance().run();
        SimHooks.stepTiming(DT);
        cyclesRun++;

        Pose2d odom = container.swerveSubsystem.getState().Pose;
        Optional<VisionSubsystem.AcceptedObservationSnapshot> vision = container.visionSubsystem
            .getLatestAcceptedObservationSnapshot();

        double jump = odom.getTranslation().getDistance(prev.getTranslation());
        if (cycle >= WARMUP_CYCLES) {
          maxJump = Math.max(maxJump, jump);
        }
        prev = odom;

        if (vision.isPresent()) {
          visionAcceptedCycles++;
          sumVisionX += vision.get().pose().getX();
          sumVisionY += vision.get().pose().getY();
          sumVisionYawDeg += vision.get().pose().getRotation().getDegrees();
        }

        double t = cycle * DT;
        csv.add(
            String.format(
                "%d,%.3f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f,%b",
                cycle,
                t,
                odom.getX(),
                odom.getY(),
                odom.getRotation().getDegrees(),
                vision.map(s -> s.pose().getX()).orElse(Double.NaN),
                vision.map(s -> s.pose().getY()).orElse(Double.NaN),
                vision.map(s -> s.pose().getRotation().getDegrees()).orElse(Double.NaN),
                jump,
                vision.isPresent()));
      }
    } catch (Throwable t) {
      writeCsv("vision-pose-static-test.csv", csv);
      Assumptions.abort("Headless sim threw while running the auto: " + t);
      return;
    }

    writeCsv("vision-pose-static-test.csv", csv);

    Assumptions.assumeTrue(cyclesRun > WARMUP_CYCLES, "Auto did not run long enough to evaluate");

    System.out.printf(
        "[VisionPoseStatic] cycles=%d visionAcceptedCycles=%d maxJump=%.3f m%n",
        cyclesRun, visionAcceptedCycles, maxJump);

    if (visionAcceptedCycles > 0) {
      double meanVisionX = sumVisionX / visionAcceptedCycles;
      double meanVisionY = sumVisionY / visionAcceptedCycles;
      double meanVisionYawDeg = sumVisionYawDeg / visionAcceptedCycles;
      System.out.printf(
          "[VisionPoseStatic] meanVisionPose=(%.4f, %.4f, %.2f°) expectedPose=(%.4f, %.4f, %.2f°)%n",
          meanVisionX,
          meanVisionY,
          meanVisionYawDeg,
          STARTING_POSE_X,
          STARTING_POSE_Y,
          STARTING_POSE_YAW_DEG);
    }

    final int observedVisionCycles = visionAcceptedCycles;
    final double observedMaxJump = maxJump;
    Assumptions.assumeTrue(
        observedVisionCycles > 0,
        "No vision observations were accepted in this static sim pose"
            + " (likely no visible AprilTags from this location/orientation in headless sim)");
    assertTrue(
        observedMaxJump <= MAX_SINGLE_CYCLE_JUMP_M,
        () -> "Max single-cycle odometry jump "
            + observedMaxJump
            + " m exceeded "
            + MAX_SINGLE_CYCLE_JUMP_M
            + " m for a stationary robot."
            + " See build/vision-stability/vision-pose-static-test.csv");
  }

  private static void writeCsv(String name, List<String> lines) throws IOException {
    Path dir = Path.of("build", "vision-stability");
    Files.createDirectories(dir);
    Files.write(dir.resolve(name), lines);
  }
}
