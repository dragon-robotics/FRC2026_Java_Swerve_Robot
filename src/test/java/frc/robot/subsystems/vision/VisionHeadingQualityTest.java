package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class VisionHeadingQualityTest {
  @ParameterizedTest
  @CsvSource({
    "5, 0, false",
    "-5, 0, false",
    "5.0001, 0, true",
    "-5.0001, 0, true",
    "179, -179, false",
    "-179, 179, false",
    "176, -179, false",
    "175.9, -179, true",
    "-175.9, 179, true"
  })
  void comparesWrappedCaptureHeadingAndPreservesTheInclusiveBoundary(
      double visionDegrees, double captureDegrees, boolean rejected) {
    for (var type : PoseObservationType.values()) {
      var observation =
          new PoseObservation(
              1,
              new Pose3d(new Pose2d(4, 4, Rotation2d.fromDegrees(visionDegrees))),
              .05,
              2,
              2,
              type,
              new int[] {29, 30});
      var reference = new Pose2d(3, 3, Rotation2d.fromDegrees(captureDegrees));
      var result = VisionSubsystem.rejectionReason(observation, reference, true);
      assertEquals(rejected, result.isPresent());
      if (rejected) assertEquals("HEADING_DELTA", result.orElseThrow());
      assertTrue(
          VisionSubsystem.rejectionReason(observation, reference, false).isEmpty(),
          "An unaligned gyro cannot veto initial field localization");
    }
  }
}
