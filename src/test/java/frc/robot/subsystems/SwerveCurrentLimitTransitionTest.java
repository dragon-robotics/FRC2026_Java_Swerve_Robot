package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import org.junit.jupiter.api.Test;

class SwerveCurrentLimitTransitionTest {

  @Test
  void autoDriveCurrentLimitsLeaveStatorProtectionDisabled() {
    CurrentLimitsConfigs limits = CommandSwerveDrivetrain.createAutoDriveCurrentLimits();

    assertFalse(limits.StatorCurrentLimitEnable);
    assertTrue(limits.SupplyCurrentLimitEnable);
    assertEquals(60.0, limits.SupplyCurrentLimit, 1e-9);
    assertEquals(40.0, limits.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.25, limits.SupplyCurrentLowerTime, 1e-9);
  }

  @Test
  void teleopDriveCurrentLimitsEnableThe80AmpStatorLimit() {
    CurrentLimitsConfigs limits = CommandSwerveDrivetrain.createTeleopDriveCurrentLimits();

    assertTrue(limits.StatorCurrentLimitEnable);
    assertEquals(80.0, limits.StatorCurrentLimit, 1e-9);
    assertTrue(limits.SupplyCurrentLimitEnable);
    assertEquals(60.0, limits.SupplyCurrentLimit, 1e-9);
    assertEquals(40.0, limits.SupplyCurrentLowerLimit, 1e-9);
    assertEquals(0.25, limits.SupplyCurrentLowerTime, 1e-9);
  }
}
