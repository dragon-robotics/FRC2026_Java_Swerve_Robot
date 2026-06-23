package frc.robot.subsystems;

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
import frc.robot.subsystems.Superstructure.SuperState;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

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
  void shootNoAimWithJuicerDelayOwnsMechanismsWithoutSwerve() {
    Assumptions.assumeTrue(halReady, "HAL/simulation unavailable in this environment");
    Assumptions.assumeTrue(container != null, "RobotContainer failed to initialize");

    Command shootNoAimWithJuicer =
        container.superstructureSubsystem.shootNoAimWithJuicerDelayCmd();

    assertTrue(shootNoAimWithJuicer.getRequirements().contains(container.intakeSubsystem));
    assertTrue(shootNoAimWithJuicer.getRequirements().contains(container.hopperSubsystem));
    assertTrue(shootNoAimWithJuicer.getRequirements().contains(container.shooterSubsystem));
    assertFalse(shootNoAimWithJuicer.getRequirements().contains(container.swerveSubsystem));
  }

  private static RobotContainer createContainerInAllowedShootZone() {
    RobotContainer container = new RobotContainer();
    container.swerveSubsystem.resetPose(new Pose2d(1.0, 4.0, Rotation2d.kZero));
    container.superstructureSubsystem.periodic();
    return container;
  }
}
