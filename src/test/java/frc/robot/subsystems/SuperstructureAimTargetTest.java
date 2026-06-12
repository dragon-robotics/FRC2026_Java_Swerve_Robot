package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;
import org.junit.jupiter.api.Test;

class SuperstructureAimTargetTest {

  @Test
  void redRightAimPointsMirrorRedLeftAcrossHorizontalCenterline() {
    assertEquals(
        FieldConstants.FIELD_WIDTH - FieldConstants.AimPoints.RED_LEFT_PURGE_POINT.getY(),
        FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT.getY());
    assertEquals(
        FieldConstants.FIELD_WIDTH - FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT.getY(),
        FieldConstants.AimPoints.RED_RIGHT_SHOOT_POINT.getY());

    assertNotEquals(
        FieldConstants.AimPoints.RED_LEFT_PURGE_POINT,
        FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT);
    assertNotEquals(
        FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT,
        FieldConstants.AimPoints.RED_RIGHT_SHOOT_POINT);
  }

  @Test
  void redAllianceNeutralZonesUseFlippedLeftRightAimPoints() {
    assertEquals(
        FieldConstants.AimPoints.RED_RIGHT_SHOOT_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_LEFT_SHOOT, DriverStation.Alliance.Red));
    assertEquals(
        FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_RIGHT_SHOOT, DriverStation.Alliance.Red));
    assertEquals(
        FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_LEFT_PURGE, DriverStation.Alliance.Red));
    assertEquals(
        FieldConstants.AimPoints.RED_LEFT_PURGE_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_RIGHT_PURGE, DriverStation.Alliance.Red));
  }

  @Test
  void blueAllianceNeutralZonesKeepStandardLeftRightAimPoints() {
    assertEquals(
        FieldConstants.AimPoints.BLUE_LEFT_SHOOT_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_LEFT_SHOOT, DriverStation.Alliance.Blue));
    assertEquals(
        FieldConstants.AimPoints.BLUE_RIGHT_SHOOT_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_RIGHT_SHOOT, DriverStation.Alliance.Blue));
    assertEquals(
        FieldConstants.AimPoints.BLUE_LEFT_PURGE_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_LEFT_PURGE, DriverStation.Alliance.Blue));
    assertEquals(
        FieldConstants.AimPoints.BLUE_RIGHT_PURGE_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_RIGHT_PURGE, DriverStation.Alliance.Blue));
  }

  @Test
  void defaultsToAllianceHubWhenZoneUnknownOrUnconfirmed() {
    Translation2d redHub = FieldConstants.Hub.RED_CENTER_POSE;
    Translation2d blueHub = FieldConstants.Hub.BLUE_CENTER_POSE;

    assertEquals(
        redHub,
        Superstructure.resolveAimTargetForZone(false, FieldZones.NEUTRAL_LEFT_SHOOT, DriverStation.Alliance.Red));
    assertEquals(
        redHub,
        Superstructure.resolveAimTargetForZone(true, null, DriverStation.Alliance.Red));
    assertEquals(
        blueHub,
        Superstructure.resolveAimTargetForZone(false, FieldZones.NEUTRAL_RIGHT_PURGE, DriverStation.Alliance.Blue));
    assertEquals(
        blueHub,
        Superstructure.resolveAimTargetForZone(true, null, DriverStation.Alliance.Blue));
  }
}
