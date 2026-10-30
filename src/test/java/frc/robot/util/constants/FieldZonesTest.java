package frc.robot.util.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.constants.FieldConstants.FieldZones;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class FieldZonesTest {
  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @AfterAll
  static void shutdownHal() {
    HAL.shutdown();
  }

  @Test
  void formerPurgeAreasUseNeutralShotsForBothAlliances() {
    for (Alliance alliance : Alliance.values()) {
      assertZone(alliance, 6.0, 0.5, FieldZones.NEUTRAL_RIGHT_SHOOT);
      assertZone(alliance, 6.0, 7.5, FieldZones.NEUTRAL_LEFT_SHOOT);
    }
  }

  @Test
  void shootingStripsContinueAcrossMidfieldIntoOpposingHalf() {
    for (Alliance alliance : Alliance.values()) {
      for (double x : new double[] {7.0, 8.2705, 9.5, 10.8}) {
        assertZone(alliance, x, 2.5, FieldZones.NEUTRAL_RIGHT_SHOOT);
        assertZone(alliance, x, 5.5, FieldZones.NEUTRAL_LEFT_SHOOT);
      }
    }
  }

  @Test
  void stripsKeepNearNeutralClearanceAndExcludeCenterLane() {
    for (Alliance alliance : Alliance.values()) {
      // The near edge remains 5.6416 meters in alliance-normalized X.
      assertZone(alliance, 5.642, 2.5, FieldZones.NEUTRAL_RIGHT_SHOOT);
      assertZone(alliance, 10.899, 2.5, FieldZones.NEUTRAL_RIGHT_SHOOT);
      assertZone(alliance, 5.641, 2.5, FieldZones.ALLIANCE_RIGHT_BUMP);
      assertZone(alliance, 10.900, 2.5, FieldZones.OPPONENT_RIGHT_BUMP);
      assertZone(alliance, 7.0, 4.0, FieldZones.NEUTRAL_RIGHT);
      assertZone(alliance, 9.5, 4.1, FieldZones.NEUTRAL_LEFT);
    }
  }

  @Test
  void shootingStripsExtendThroughOpponentAllianceZoneWithWallClearance() {
    for (Alliance alliance : Alliance.values()) {
      for (double x : new double[] {13.5, 15.0, 16.121}) {
        assertZone(alliance, x, 2.5, FieldZones.NEUTRAL_RIGHT_SHOOT);
        assertZone(alliance, x, 5.5, FieldZones.NEUTRAL_LEFT_SHOOT);
      }
      assertZone(alliance, 16.122, 2.5, FieldZones.OPPONENT_RIGHT);
      assertZone(alliance, 16.122, 5.5, FieldZones.OPPONENT_LEFT);
      assertZone(alliance, 15.0, 4.0, FieldZones.OPPONENT_RIGHT);
      assertZone(alliance, 15.0, 4.1, FieldZones.OPPONENT_LEFT);
    }
  }

  @Test
  void opposingBumpsOverrideShootingStripsButNearbyTrenchesStillAllowShots() {
    for (Alliance alliance : Alliance.values()) {
      for (double x : new double[] {10.900, 11.5, 12.5}) {
        assertZone(alliance, x, 2.5, FieldZones.OPPONENT_RIGHT_BUMP);
        assertZone(alliance, x, 5.5, FieldZones.OPPONENT_LEFT_BUMP);
        assertZone(alliance, x, 0.5, FieldZones.NEUTRAL_RIGHT_SHOOT);
        assertZone(alliance, x, 7.5, FieldZones.NEUTRAL_LEFT_SHOOT);
      }
      assertZone(alliance, 10.899, 2.5, FieldZones.NEUTRAL_RIGHT_SHOOT);
      assertZone(alliance, 13.5, 2.5, FieldZones.NEUTRAL_RIGHT_SHOOT);
    }
  }

  private static void assertZone(Alliance alliance, double x, double y, FieldZones expected) {
    Pose2d pose =
        alliance == Alliance.Red
            ? new Pose2d(
                FieldConstants.FIELD_LENGTH - x, FieldConstants.FIELD_WIDTH - y, Rotation2d.kZero)
            : new Pose2d(x, y, Rotation2d.kZero);
    assertEquals(expected, FieldZones.fromPose(pose, alliance), alliance + " at " + x + ", " + y);
  }
}
