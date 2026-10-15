package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import java.util.Optional;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class DefaultDriveHeadingHandoffTest {
  private static RobotContainer container;

  @BeforeAll
  static void initializeRobot() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    container = new RobotContainer();
    DogLog.setOptions(new DogLogOptions().withUseLogThread(false));
  }

  @AfterAll
  static void shutdownRobot() {
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    HAL.shutdown();
  }

  @Test
  void teleopReacquiresFinalAutoHeadingForBothAlliances() {
    Command drive =
        container.superstructureSubsystem.defaultDrive(
            () -> 0.0, () -> 0.0, () -> 0.0, () -> false, () -> false);

    for (AllianceStationID alliance :
        new AllianceStationID[] {AllianceStationID.Blue1, AllianceStationID.Red1}) {
      DriverStationSim.setAllianceStationId(alliance);
      DriverStationSim.setAutonomous(true);
      DriverStationSim.notifyNewData();
      // Shooting cached 35 degrees, then the final auto path turned to 110 degrees.
      container.superstructureSubsystem.setCurrentHeading(
          Optional.of(Rotation2d.fromDegrees(35.0)));
      container.swerveSubsystem.resetPose(new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(110.0)));

      DriverStationSim.setAutonomous(false);
      DriverStationSim.notifyNewData();
      drive.initialize();
      drive.execute();

      double expectedDegrees = alliance == AllianceStationID.Red1 ? -70.0 : 110.0;
      assertEquals(
          expectedDegrees,
          container.superstructureSubsystem.getCurrentHeading().orElseThrow().getDegrees(),
          1e-6,
          "Teleop must hold the final auto heading instead of the previous shooting target");
      drive.end(true);
    }
  }

  @Test
  void translatingKeepsCapturedHeadingUntilDriverRotates() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    double[] rotationInput = {0.0};
    Command drive =
        container.superstructureSubsystem.defaultDrive(
            () -> 0.5, () -> 0.25, () -> rotationInput[0], () -> false, () -> false);
    container.swerveSubsystem.resetPose(new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(110.0)));
    drive.initialize();
    drive.execute();

    // A heading disturbance while translating must not replace the held target.
    container.swerveSubsystem.resetPose(new Pose2d(2.1, 3.1, Rotation2d.fromDegrees(115.0)));
    drive.execute();
    assertEquals(
        110.0,
        container.superstructureSubsystem.getCurrentHeading().orElseThrow().getDegrees(),
        1e-6);

    rotationInput[0] = 0.5;
    drive.execute();
    assertTrue(container.superstructureSubsystem.getCurrentHeading().isEmpty());
    drive.end(true);
  }
}
