package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionIOPhotonVision.TagDistanceConfidenceMode.ALL_TAG_AVERAGE;
import static frc.robot.subsystems.vision.VisionIOPhotonVision.TagDistanceConfidenceMode.MAX_TAG_DISTANCE;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class VisionDistanceConfidencePolicyTest {

  @Test
  void mixedNearAndFarTagsShowsConfidenceDifferenceAcrossSupportedPolicies() {
    double totalDistanceAll = 9.0;
    int distanceSampleCountAll = 2;
    double maxDistanceAll = 7.0;

    assertArrayEquals(
        new VisionIOPhotonVision.TagDistanceConfidenceMode[] {ALL_TAG_AVERAGE, MAX_TAG_DISTANCE},
        VisionIOPhotonVision.TagDistanceConfidenceMode.values());
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
}
