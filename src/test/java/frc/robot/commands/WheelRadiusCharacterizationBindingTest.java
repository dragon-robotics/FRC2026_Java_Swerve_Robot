package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.util.constants.OperatorConstants;
import java.util.Optional;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class WheelRadiusCharacterizationBindingTest {
  private static RobotContainer container;

  @BeforeAll
  static void initializeRobot() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setJoystickButtonCount(OperatorConstants.TEST_PORT, 10);
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
  void rightBumperRunsInTeleopAndTestAndClearsThePreviousDriverHeadingOnRelease() {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.run();
    scheduler.run();

    for (boolean testMode : new boolean[] {false, true}) {
      DriverStationSim.setTest(testMode);
      DriverStationSim.notifyNewData();
      container.superstructureSubsystem.setCurrentHeading(
          Optional.of(Rotation2d.fromDegrees(35.0)));

      setBumper(true);
      scheduler.run();
      assertTrue(running(), "Right bumper should start characterization in teleop/test");
      assertTrue(
          container
              .swerveSubsystem
              .getCurrentCommand()
              .getName()
              .contains("WheelRadiusCharacterizationCmd"));

      setBumper(false);
      scheduler.run();
      assertFalse(running());
      assertTrue(
          container.superstructureSubsystem.getCurrentHeading().isEmpty(),
          "Driver heading hold must not rotate back to its pre-characterization target");
      scheduler.run();
    }

    DriverStationSim.setTest(false);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    setBumper(true);
    scheduler.run();
    assertFalse(running(), "The test controller must not start characterization in autonomous");
    setBumper(false);
  }

  private static void setBumper(boolean pressed) {
    DriverStationSim.setJoystickButton(
        OperatorConstants.TEST_PORT, XboxController.Button.kRightBumper.value, pressed);
    DriverStationSim.notifyNewData();
  }

  private static boolean running() {
    return NetworkTableInstance.getDefault()
        .getTable("Robot/Swerve/WheelRadiusCharacterization")
        .getEntry("Running")
        .getBoolean(false);
  }
}
