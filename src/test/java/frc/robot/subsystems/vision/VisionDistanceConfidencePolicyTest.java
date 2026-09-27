package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionIOPhotonVision.TagDistanceConfidenceMode.ALL_TAG_AVERAGE;
import static frc.robot.subsystems.vision.VisionIOPhotonVision.TagDistanceConfidenceMode.MAX_TAG_DISTANCE;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class VisionDistanceConfidencePolicyTest {

  @Test
  void mixedNearAndFarTagsShowsConfidenceDifferenceAcrossSupportedPolicies() {
    double totalDistanceAll = 9.0;
    int distanceSampleCountAll = 2;
    double maxDistanceAll = 7.0;

    assertEquals(
        4.5,
        VisionIOPhotonVision.confidenceDistanceForTest(
            ALL_TAG_AVERAGE, totalDistanceAll, distanceSampleCountAll, maxDistanceAll),
        1e-9);
    assertEquals(
        7.0,
        VisionIOPhotonVision.confidenceDistanceForTest(
            MAX_TAG_DISTANCE, totalDistanceAll, distanceSampleCountAll, maxDistanceAll),
        1e-9);
  }

  @Test
  void missingDistanceSamplesCannotLookLikeACloseTag() {
    assertEquals(
        Double.POSITIVE_INFINITY,
        VisionIOPhotonVision.confidenceDistanceForTest(ALL_TAG_AVERAGE, 0, 0, 0));
  }

  @Test
  void averageDistanceGateIncludesSevenMetersAndRejectsAboveIt() {
    for (double distance : new double[] {7.0, Math.nextUp(7.0)}) {
      var observation =
          new VisionIO.PoseObservation(
              1,
              new edu.wpi.first.math.geometry.Pose3d(
                  4, 4, 0, new edu.wpi.first.math.geometry.Rotation3d()),
              .05,
              3,
              distance,
              VisionIO.PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
              new int[] {2, 3, 4});
      assertEquals(distance > 7, VisionSubsystem.rejectionReason(observation).isPresent());
      if (distance > 7)
        assertTrue(
            VisionSubsystem.rejectionReason(observation).orElseThrow().startsWith("DISTANCE="));
    }
  }

  @Test
  void distantTwoTagCoprocessorPoseIsRejectedBeforeFusion() {
    var observation =
        new VisionIO.PoseObservation(
            1,
            new edu.wpi.first.math.geometry.Pose3d(
                6.884578,
                .132107,
                .351113,
                new edu.wpi.first.math.geometry.Rotation3d(0, 0, 1.801843)),
            .59894,
            2,
            6.63494,
            VisionIO.PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR,
            new int[] {29, 30});

    assertTrue(
        VisionSubsystem.rejectionReason(observation).isPresent(),
        "The logged 1.8 m jump must not qualify as usable two-tag vision");
  }

  @Test
  void twoTagRangeGatePreservesSixMeterBoundaryAndOtherSolvers() {
    for (var type : VisionIO.PoseObservationType.values()) {
      for (double distance : new double[] {6, Math.nextUp(6.0)}) {
        var observation =
            new VisionIO.PoseObservation(
                1,
                new edu.wpi.first.math.geometry.Pose3d(
                    4, 4, 0, new edu.wpi.first.math.geometry.Rotation3d()),
                .05,
                2,
                distance,
                type,
                new int[] {29, 30});
        assertEquals(
            type == VisionIO.PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR && distance > 6,
            VisionSubsystem.rejectionReason(observation).isPresent(),
            "A coprocessor range limit must not discard gyro-constrained two-tag poses");
      }
    }
  }
}
