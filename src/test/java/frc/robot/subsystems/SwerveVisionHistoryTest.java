package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.generated.TunerConstants;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class SwerveVisionHistoryTest {
  private static CommandSwerveDrivetrain swerve;
  private static double beforeConstruction;

  @BeforeAll
  static void startDrivetrain() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.resumeTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    beforeConstruction = Timer.getFPGATimestamp() - 1.0;
    swerve = TunerConstants.createDrivetrain();
    waitForRecentHistory();
  }

  @AfterAll
  static void stopDrivetrain() {
    if (swerve != null) swerve.close();
  }

  @Test
  void neverClampsMissingHistoryToAnEndpoint() {
    assertTrue(swerve.samplePoseAt(beforeConstruction).isEmpty(), "No pre-construction history");
    assertTrue(swerve.samplePoseAt(Timer.getFPGATimestamp() - 2.0).isEmpty(), "No expired history");
    assertTrue(swerve.samplePoseAt(Timer.getFPGATimestamp() + 1.0).isEmpty(), "No future history");
    assertTrue(swerve.samplePoseAt(Double.NaN).isEmpty(), "No invalid timestamps");
    assertTrue(swerve.samplePoseAt(Double.POSITIVE_INFINITY).isEmpty());
    assertTrue(swerve.samplePoseAt(Double.NEGATIVE_INFINITY).isEmpty());
    assertTrue(swerve.samplePoseAt(waitForRecentHistory()).isPresent());
  }

  @Test
  void resetDiscardsEarlierCaptureHistoryAndKeepsNewPoseSamples() {
    double captureBeforeReset = waitForRecentHistory();
    assertTrue(swerve.samplePoseAt(captureBeforeReset).isPresent());

    Pose2d resetPose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(25));
    swerve.resetPose(resetPose);
    double captureAfterReset = waitForRecentHistory();

    assertTrue(
        swerve.samplePoseAt(captureBeforeReset).isEmpty(),
        "A buffered camera frame from before reset must not match the new coordinate origin");
    Pose2d sampled = swerve.samplePoseAt(captureAfterReset).orElseThrow();
    assertEquals(3.0, sampled.getX(), .01);
    assertEquals(4.0, sampled.getY(), .01);
    assertEquals(25.0, sampled.getRotation().getDegrees(), .1);
  }

  @Test
  void registeringTelemetryPreservesHistoryAndTheUserCallback() {
    AtomicInteger updates = new AtomicInteger();
    swerve.registerTelemetry(state -> updates.incrementAndGet());
    waitUntil(() -> updates.get() > 0, "External drivetrain logging must still receive states");
    assertTrue(swerve.samplePoseAt(waitForRecentHistory()).isPresent());
    assertTrue(swerve.samplePoseAt(Timer.getFPGATimestamp() + 1).isEmpty());

    swerve.registerTelemetry(null);
    double captureAfterUnregister = Timer.getFPGATimestamp();
    waitUntil(
        () -> swerve.samplePoseAt(captureAfterUnregister).isPresent(),
        "History must continue advancing after external telemetry is unregistered");
  }

  @Test
  void manualHeadingSeedDiscardsHistoryWithoutChangingTranslation() {
    swerve.resetPose(new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(25)));
    swerve.setOperatorPerspectiveForward(Rotation2d.kZero);
    double captureBeforeSeed = waitForRecentHistory();
    assertTrue(swerve.samplePoseAt(captureBeforeSeed).isPresent());

    swerve.seedFieldCentric();
    double captureAfterSeed = waitForRecentHistory();

    assertTrue(swerve.samplePoseAt(captureBeforeSeed).isEmpty());
    Pose2d sampled = swerve.samplePoseAt(captureAfterSeed).orElseThrow();
    assertEquals(3.0, sampled.getX(), .01);
    assertEquals(4.0, sampled.getY(), .01);
    assertEquals(0.0, sampled.getRotation().getDegrees(), .1);
  }

  private static double waitForRecentHistory() {
    double[] capture = new double[1];
    waitUntil(
        () -> {
          capture[0] = Timer.getFPGATimestamp() - .05;
          return swerve.samplePoseAt(capture[0]).isPresent();
        },
        "Native odometry must provide capture-time history");
    return capture[0];
  }

  private static void waitUntil(BooleanSupplier condition, String message) {
    long deadline = System.nanoTime() + 2_000_000_000L;
    boolean satisfied = condition.getAsBoolean();
    while (!satisfied && System.nanoTime() < deadline) {
      Timer.delay(.005);
      satisfied = condition.getAsBoolean();
    }
    assertTrue(
        satisfied,
        () ->
            message
                + "; FPGA="
                + Timer.getFPGATimestamp()
                + ", CTRE now="
                + Utils.getCurrentTimeSeconds()
                + ", latest odometry="
                + swerve.getState().Timestamp);
  }
}
