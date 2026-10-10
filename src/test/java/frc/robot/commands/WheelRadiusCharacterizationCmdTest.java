package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class WheelRadiusCharacterizationCmdTest {
  private final Subsystem drivetrain = new Subsystem() {};
  private final NetworkTable table =
      NetworkTableInstance.getDefault().getTable("Robot/Swerve/WheelRadiusCharacterization");
  private SwerveDriveState state;
  private SwerveRequest lastRequest;
  private WheelRadiusCharacterizationCmd command;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DogLog.setOptions(new DogLogOptions().withUseLogThread(false).withNtPublish(false));
  }

  @AfterAll
  static void shutdownHal() {
    SimHooks.resumeTiming();
    HAL.shutdown();
  }

  @BeforeEach
  void createCommand() {
    state = new SwerveDriveState();
    state.Pose = new Pose2d();
    state.Speeds = new ChassisSpeeds();
    state.ModuleStates = new SwerveModuleState[4];
    state.ModuleTargets = new SwerveModuleState[4];
    state.ModulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      state.ModuleStates[i] = new SwerveModuleState();
      state.ModuleTargets[i] = new SwerveModuleState();
      state.ModulePositions[i] = new SwerveModulePosition(10.0 + i, Rotation2d.kZero);
    }
    command =
        new WheelRadiusCharacterizationCmd(
            drivetrain,
            () -> state,
            request -> lastRequest = request,
            moduleConstants(0.1),
            moduleConstants(0.2),
            moduleConstants(0.1),
            moduleConstants(0.2));
    command.initialize();
  }

  @Test
  void rampsRotationWithoutTranslationAndWaitsForModuleAlignment() {
    SimHooks.stepTiming(0.5);
    command.execute();

    var request = (SwerveRequest.RobotCentric) lastRequest;
    assertEquals(0.0, request.VelocityX);
    assertEquals(0.0, request.VelocityY);
    assertEquals(0.025, request.RotationalRate, 1e-6);
    assertEquals(0.0, value("GyroDeltaRadians"));
    assertTrue(flag("Running"));
    assertFalse(flag("Valid"));
    assertTrue(command.getRequirements().contains(drivetrain));

    SimHooks.stepTiming(5.0);
    command.execute();
    assertEquals(0.25, ((SwerveRequest.RobotCentric) lastRequest).RotationalRate, 1e-6);
  }

  @Test
  void reportsEffectiveRadiusFromAllWheelsAndRawHeading() {
    startMeasurement();
    for (int step = 1; step <= 4; step++) {
      // A 0.5 m drive-base radius with 0.04 m wheels travels 25 pi wheel radians per turn.
      state.RawHeading = Rotation2d.fromRadians(step * Math.PI / 2.0);
      state.Pose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(step * 7.0));
      setWheelDistances(step * 0.625 * Math.PI, -step * 1.25 * Math.PI);
      command.execute();
    }

    assertEquals(2.0 * Math.PI, value("GyroDeltaRadians"), 1e-9);
    assertEquals(25.0 * Math.PI, value("WheelDeltaRadians"), 1e-9);
    assertEquals(0.04, value("RadiusMeters"), 1e-9);
    assertEquals(1.574803149606299, value("RadiusInches"), 1e-9);
    assertTrue(flag("Valid"));
  }

  @Test
  void handlesHeadingWrapWithoutCountingAnExtraRevolution() {
    state.RawHeading = Rotation2d.fromDegrees(179.0);
    startMeasurement();
    state.RawHeading = Rotation2d.fromDegrees(-179.0);
    setWheelDistances(0.1, -0.2);
    command.execute();

    assertEquals(0.034906585039887, value("GyroDeltaRadians"), 1e-9);
    assertEquals(0.017453292519943, value("RadiusMeters"), 1e-9);
    assertFalse(flag("Valid"));
  }

  @Test
  void stoppingSamplesTheFinalMotionAndLeavesTheResultVisible() {
    startMeasurement();
    for (int step = 1; step <= 3; step++) {
      state.RawHeading = Rotation2d.fromRadians(step * Math.PI / 2.0);
      setWheelDistances(step * 0.625 * Math.PI, -step * 1.25 * Math.PI);
      command.execute();
    }
    state.RawHeading = Rotation2d.kZero;
    setWheelDistances(2.5 * Math.PI, -5.0 * Math.PI);
    command.end(true);

    assertStopped();
    assertEquals(0.04, value("RadiusMeters"), 1e-9);
    assertTrue(flag("Valid"));
    assertFalse(flag("Running"));

    command.initialize();
    assertEquals(0.0, value("RadiusMeters"));
    assertFalse(flag("Valid"));
    startMeasurement();
    state.RawHeading = Rotation2d.fromDegrees(90.0);
    setWheelDistances(3.125 * Math.PI, -6.25 * Math.PI);
    command.execute();
    assertEquals(0.04, value("RadiusMeters"), 1e-9);
    assertFalse(flag("Valid"));
  }

  @Test
  void stationaryWheelsDoNotProduceAnInfiniteRadiusOrValidResult() {
    startMeasurement();
    for (int step = 1; step <= 4; step++) {
      state.RawHeading = Rotation2d.fromRadians(step * Math.PI / 2.0);
      command.execute();
    }
    command.end(true);

    assertEquals(0.0, value("RadiusMeters"));
    assertFalse(flag("Valid"));
  }

  @Test
  void releasingBeforeAlignmentStopsWithoutPublishingAResult() {
    SimHooks.stepTiming(0.5);
    command.execute();
    command.end(true);

    assertStopped();
    assertEquals(0.0, value("RadiusMeters"));
    assertFalse(flag("Valid"));
    assertFalse(flag("Running"));
  }

  private void startMeasurement() {
    SimHooks.stepTiming(1.01);
    command.execute();
  }

  private void assertStopped() {
    assertTrue(lastRequest instanceof SwerveRequest.RobotCentric);
    var request = (SwerveRequest.RobotCentric) lastRequest;
    assertEquals(0.0, request.VelocityX);
    assertEquals(0.0, request.VelocityY);
    assertEquals(0.0, request.RotationalRate);
  }

  private void setWheelDistances(double leftDelta, double rightDelta) {
    for (int i = 0; i < 4; i++) {
      state.ModulePositions[i].distanceMeters = 10.0 + i + (i % 2 == 0 ? leftDelta : rightDelta);
    }
  }

  private double value(String key) {
    return table.getEntry(key).getDouble(Double.NaN);
  }

  private boolean flag(String key) {
    assertTrue(table.getEntry(key).exists(), "Missing DogLog field: " + key);
    return table.getEntry(key).getBoolean(false);
  }

  private static SwerveModuleConstants<?, ?, ?> moduleConstants(double radiusMeters) {
    return new SwerveModuleConstants<>()
        .withLocationX(0.3)
        .withLocationY(0.4)
        .withWheelRadius(radiusMeters);
  }
}
