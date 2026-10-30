package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure.ShootMode;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.util.constants.OperatorConstants;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class SuperstructureCommandRequirementsTest {

  private static boolean halReady = false;
  private static RobotContainer container;

  @BeforeAll
  static void setUpHal() {
    try {
      halReady = HAL.initialize(500, 0);
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.notifyNewData();
      container = createContainerInAllowedShootZone();
    } catch (Throwable t) {
      halReady = false;
      container = null;
    }
  }

  @AfterAll
  static void tearDownHal() {
    try {
      CommandScheduler.getInstance().cancelAll();
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    } catch (Throwable ignored) {
      // best effort
    }
    if (halReady) {
      HAL.shutdown();
    }
  }

  @Test
  void shootNoAimDoesNotRequireSwerve() {
    Assumptions.assumeTrue(halReady, "HAL/simulation unavailable in this environment");
    Assumptions.assumeTrue(container != null, "RobotContainer failed to initialize");

    Command shootNoAim = container.superstructureSubsystem.setStateCmd(SuperState.SHOOT_NO_AIM);

    assertFalse(shootNoAim.getRequirements().contains(container.swerveSubsystem));
  }

  @Test
  void shootWithAimRequiresSwerve() {
    Assumptions.assumeTrue(halReady, "HAL/simulation unavailable in this environment");
    Assumptions.assumeTrue(container != null, "RobotContainer failed to initialize");

    Command shootWithAim = container.superstructureSubsystem.setStateCmd(SuperState.SHOOT_WITH_AIM);

    assertTrue(shootWithAim.getRequirements().contains(container.swerveSubsystem));
  }

  @Test
  void defaultShootModeCommandsManualBumperUpSetpoint() {
    Assumptions.assumeTrue(halReady, "HAL/simulation unavailable in this environment");
    Assumptions.assumeTrue(container != null, "RobotContainer failed to initialize");

    container.shooterSubsystem.setSetpoint(0.0, 1.0);
    Command defaultShoot = container.superstructureSubsystem.selectedShootModeCmd();

    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.schedule(defaultShoot);
    scheduler.run();

    assertAll(
        () -> assertEquals(2500.0, container.shooterSubsystem.getTargetRPM(), 1e-9),
        () -> assertEquals(0.0, container.shooterSubsystem.getTargetHoodAngle(), 1e-9));

    scheduler.cancel(defaultShoot);
  }

  @Test
  void shootNoAimWithJuicerDelayOwnsMechanismsWithoutSwerve() {
    Assumptions.assumeTrue(halReady, "HAL/simulation unavailable in this environment");
    Assumptions.assumeTrue(container != null, "RobotContainer failed to initialize");

    Command shootNoAimWithJuicer = container.superstructureSubsystem.shootNoAimWithJuicerDelayCmd();

    assertTrue(shootNoAimWithJuicer.getRequirements().contains(container.intakeSubsystem));
    assertTrue(shootNoAimWithJuicer.getRequirements().contains(container.hopperSubsystem));
    assertTrue(shootNoAimWithJuicer.getRequirements().contains(container.shooterSubsystem));
    assertFalse(shootNoAimWithJuicer.getRequirements().contains(container.swerveSubsystem));
  }

  @ParameterizedTest
  @CsvSource({
    "MANUAL_BUMPER_UP_FAR, 2800.0, 0.0",
    "MANUAL_TRENCH_FAR, 3500.0, 0.75",
    "MANUAL_BUMPER_UP_FULL_POWER, 5800.0, 0.0",
    "MANUAL_TRENCH_FULL_POWER, 5800.0, 0.75"
  })
  void manualModesCommandRequestedRpmAndExistingHood(String mode, double rpm, double hood) {
    Assumptions.assumeTrue(halReady && container != null, "HAL/simulation unavailable");
    try {
      container.superstructureSubsystem.setShootMode(ShootMode.valueOf(mode));
      Command shoot = container.superstructureSubsystem.selectedShootModeCmd();
      shoot.initialize();
      shoot.execute();
      assertAll(
          () -> assertEquals(rpm, container.shooterSubsystem.getTargetRPM(), 1e-9),
          () -> assertEquals(hood, container.shooterSubsystem.getTargetHoodAngle(), 1e-9));
      shoot.end(true);
    } finally {
      container.superstructureSubsystem.setShootMode(ShootMode.MANUAL_BUMPER_UP);
    }
  }

  @ParameterizedTest
  @CsvSource({
    "3, MANUAL_BUMPER_UP_FAR",
    "6, MANUAL_TRENCH_FAR",
    "5, MANUAL_BUMPER_UP_FULL_POWER",
    "12, MANUAL_TRENCH_FULL_POWER"
  })
  void operatorCombosToggleManualModesWithoutJuicer(int buttons, String mode) {
    Assumptions.assumeTrue(halReady && container != null, "HAL/simulation unavailable");
    CommandScheduler scheduler = CommandScheduler.getInstance();
    int port = OperatorConstants.OPERATOR_PORT;
    DriverStationSim.setJoystickButtonCount(port, 10);
    try {
      container.superstructureSubsystem.setShootMode(ShootMode.DEFAULT_SHOOT_WITH_AIM);
      for (int press = 0; press < 2; press++) {
        DriverStationSim.setJoystickButtons(port, 0);
        DriverStationSim.notifyNewData();
        scheduler.run();
        DriverStationSim.setJoystickButtons(port, buttons);
        DriverStationSim.notifyNewData();
        scheduler.run();
        assertEquals(
            press == 0 ? mode : ShootMode.DEFAULT_SHOOT_WITH_AIM.name(),
            container.superstructureSubsystem.getShootMode().name());
        assertFalse(container.intakeSubsystem.getDesiredState() == IntakeState.JUICER);
      }
    } finally {
      DriverStationSim.setJoystickButtons(port, 0);
      DriverStationSim.notifyNewData();
      scheduler.run();
      scheduler.cancelAll();
      container.superstructureSubsystem.setShootMode(ShootMode.MANUAL_BUMPER_UP);
    }
  }

  private static RobotContainer createContainerInAllowedShootZone() {
    RobotContainer container = new RobotContainer();
    container.swerveSubsystem.resetPose(new Pose2d(1.0, 4.0, Rotation2d.kZero));
    container.superstructureSubsystem.periodic();
    return container;
  }
}
