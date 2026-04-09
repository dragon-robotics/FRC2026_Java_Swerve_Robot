// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.io.SignalRegistry;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.List;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // ── GC tracking ───────────────────────────────────────────────────────────
  private final List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();

  private long lastGcCount = 0;
  private long lastGcTimeMs = 0;

  // ── Loop overrun tracking ─────────────────────────────────────────────────
  private int overrunCount = 0;

  public Robot() {
    // super(0.025); // 25ms loop period — gives 5ms headroom over default 20ms
    m_robotContainer = new RobotContainer();

    // Set brownout voltage to 5.5V
    RobotController.setBrownoutVoltage(5.5);
  }

  @Override
  public void robotPeriodic() {
    double cycleStart = Timer.getFPGATimestamp();
    DogLog.time("Perf/Total");

    // Batch-refresh all CTRE motor signals BEFORE the scheduler runs subsystem
    // periodics
    DogLog.time("Perf/SignalRefresh");
    SignalRegistry.getInstance().refreshAll();
    DogLog.timeEnd("Perf/SignalRefresh");

    DogLog.time("Perf/Scheduler");
    CommandScheduler.getInstance().run();
    DogLog.timeEnd("Perf/Scheduler");

    DogLog.timeEnd("Perf/Total");

    // ── Loop overrun detection ──────────────────────────────────────────────
    double cycleMs = (Timer.getFPGATimestamp() - cycleStart) * 1000.0;
    DogLog.log("Perf/CycleMs", cycleMs);
    if (cycleMs > 20.0) {
      overrunCount++;
    }
    DogLog.log("Perf/OverrunCount", overrunCount);

    // ── Heap ────────────────────────────────────────────────────────────────
    Runtime rt = Runtime.getRuntime();
    DogLog.log("Perf/HeapUsedMB", (rt.totalMemory() - rt.freeMemory()) / (1024.0 * 1024.0));

    // ── GC deltas ───────────────────────────────────────────────────────────
    long totalGcCount = 0;
    long totalGcTimeMs = 0;
    for (GarbageCollectorMXBean bean : gcBeans) {
      long c = bean.getCollectionCount();
      long t = bean.getCollectionTime();
      if (c >= 0)
        totalGcCount += c;
      if (t >= 0)
        totalGcTimeMs += t;
    }
    DogLog.log("Perf/GCDeltaCount", (int) (totalGcCount - lastGcCount));
    DogLog.log("Perf/GCDeltaTimeMs", (double) (totalGcTimeMs - lastGcTimeMs));
    lastGcCount = totalGcCount;
    lastGcTimeMs = totalGcTimeMs;

    // ── Power ───────────────────────────────────────────────────────────────
    DogLog.log("Perf/BatteryVoltage", RobotController.getBatteryVoltage());
    DogLog.log("Perf/BrownoutVoltage", RobotController.getBrownoutVoltage());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
