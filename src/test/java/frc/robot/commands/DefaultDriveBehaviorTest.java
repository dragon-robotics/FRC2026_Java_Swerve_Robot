package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DefaultDriveBehaviorTest {
  private static RecordingDrivetrain swerve;
  private Optional<Rotation2d> heading;
  private double lastRotationTime;
  private double rotation;
  private boolean halfSpeed;
  private boolean angleLock;
  private Optional<Rotation2d> zoneHeading;
  private Command drive;

  @BeforeAll
  static void initializeRobot() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    swerve = new RecordingDrivetrain();
  }

  @BeforeEach
  void prepareDrive() {
    CommandScheduler.getInstance().cancelAll();
    swerve.removeDefaultCommand();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    heading = Optional.of(Rotation2d.fromDegrees(35.0));
    lastRotationTime = Double.NEGATIVE_INFINITY;
    rotation = 0.0;
    halfSpeed = false;
    angleLock = false;
    zoneHeading = Optional.empty();
    swerve.lastRequest = null;
    setHeading(110.0);
    drive =
        new DefaultDriveCmd(
            swerve,
            () -> 0.55,
            () -> -0.55,
            () -> rotation,
            () -> halfSpeed,
            () -> angleLock,
            () -> heading,
            value -> heading = value,
            () -> lastRotationTime,
            value -> lastRotationTime = value,
            () -> zoneHeading);
  }

  @AfterAll
  static void shutdownRobot() {
    CommandScheduler.getInstance().cancelAll();
    swerve.removeDefaultCommand();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
    HAL.shutdown();
  }

  @Test
  void centeredRotationHoldsCapturedHeadingWhileTranslationContinues() {
    drive.initialize();
    drive.execute();
    assertFacingRequest(110.0, 1.105, -1.105);

    setHeading(115.0);
    rotation = 0.05; // Small rotation-stick noise stays inside the deadband.
    drive.execute();
    assertFacingRequest(110.0, 1.105, -1.105);

    halfSpeed = true;
    drive.execute();
    assertFacingRequest(110.0, 0.49725, -0.49725);
  }

  @Test
  void driverRotationOverridesHoldThenReleaseCapturesNewHeading() {
    drive.initialize();
    drive.execute();
    rotation = 0.55;
    drive.execute();
    var request = assertInstanceOf(SwerveRequest.FieldCentric.class, swerve.lastRequest);
    assertEquals(Math.PI / 2.0, request.RotationalRate, 1e-9);
    assertEquals(1.105, request.VelocityX, 1e-9);
    assertTrue(heading.isEmpty());

    setHeading(140.0);
    rotation = 0.0;
    SimHooks.stepTiming(0.2); // Let the existing rotation-settling window expire.
    drive.execute();
    assertFacingRequest(140.0, 1.105, -1.105);
    setHeading(145.0);
    drive.execute();
    assertFacingRequest(140.0, 1.105, -1.105);
  }

  @Test
  void zoneLockOverridesHoldAndReleaseHoldsActualHeading() {
    drive.initialize();
    angleLock = true;
    zoneHeading = Optional.of(Rotation2d.fromDegrees(45.0));
    drive.execute();
    assertFacingRequest(45.0, 1.105, -1.105);

    setHeading(60.0);
    drive.execute();
    assertFacingRequest(45.0, 1.105, -1.105);
    angleLock = false;
    drive.execute();
    assertFacingRequest(60.0, 1.105, -1.105);
  }

  @Test
  void redAllianceReacquiresHeadingInOperatorCoordinates() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    drive.initialize();
    drive.execute();
    assertFacingRequest(-70.0, 1.105, -1.105);
    setHeading(115.0);
    drive.execute();
    assertFacingRequest(-70.0, 1.105, -1.105);

    angleLock = true;
    zoneHeading = Optional.of(Rotation2d.fromDegrees(45.0));
    drive.execute();
    assertFacingRequest(-135.0, 1.105, -1.105);
  }

  @Test
  void schedulerReacquiresHeadingAfterAutoDisableAndTeleopEnable() {
    var scheduler = CommandScheduler.getInstance();
    swerve.setDefaultCommand(drive);
    scheduler.run();
    scheduler.run();
    assertFacingRequest(110.0, 1.105, -1.105);

    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    Command auto =
        Commands.run(
            () -> {
              heading = Optional.of(Rotation2d.fromDegrees(35.0));
              setHeading(160.0);
            },
            swerve);
    scheduler.schedule(auto);
    scheduler.run();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    scheduler.run();
    assertFalse(scheduler.isScheduled(auto));

    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    scheduler.run();
    scheduler.run();
    assertTrue(scheduler.isScheduled(drive));
    assertFacingRequest(160.0, 1.105, -1.105);
  }

  @Test
  void schedulerResumesDriveAtActualHeadingAfterAimCommandIsReleased() {
    var scheduler = CommandScheduler.getInstance();
    swerve.setDefaultCommand(drive);
    scheduler.run();
    scheduler.run();
    Command aim =
        Commands.run(
            () -> {
              heading = Optional.of(Rotation2d.fromDegrees(90.0));
              setHeading(85.0); // Aim was released before reaching its target.
            },
            swerve);
    scheduler.schedule(aim);
    scheduler.run();
    assertFalse(scheduler.isScheduled(drive));

    scheduler.cancel(aim);
    scheduler.run();
    scheduler.run();
    assertTrue(scheduler.isScheduled(drive));
    assertFacingRequest(85.0, 1.105, -1.105);
  }

  private void setHeading(double degrees) {
    swerve.resetPose(new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(degrees)));
  }

  private void assertFacingRequest(double degrees, double velocityX, double velocityY) {
    var request = assertInstanceOf(SwerveRequest.FieldCentricFacingAngle.class, swerve.lastRequest);
    assertEquals(degrees, request.TargetDirection.getDegrees(), 1e-6);
    assertEquals(velocityX, request.VelocityX, 1e-9);
    assertEquals(velocityY, request.VelocityY, 1e-9);
    assertEquals(0.0, request.TargetRateFeedforward, 1e-9);
  }

  // Keep real drivetrain pose readback, but record commands at the motor-control boundary.
  // This avoids asynchronous simulated motor motion changing the test's heading fixtures.
  private static class RecordingDrivetrain extends CommandSwerveDrivetrain {
    private SwerveRequest lastRequest;

    RecordingDrivetrain() {
      super(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);
    }

    @Override
    public void setControl(SwerveRequest request) {
      lastRequest = request;
    }
  }
}
