package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import org.junit.jupiter.api.Test;

class VisionRunMetadataTest {
  @Test
  void atomicRunEventPreservesTypedConfigAndQuotedBuildIdentity() {
    var event =
        VisionRunMetadata.createRunEvent(
            "run-1",
            "SIM",
            Map.of("GitBranch", "feature/\"quoted\"", "GitDirty", "true"),
            Map.of("MAX_DISTANCE", 7.5, "ENABLED", true),
            "lengthMeters=16;origin=[0,0,0,1,0,0,0]");
    assertEquals("RUN_METADATA", event.path("event").asText());
    assertEquals("feature/\"quoted\"", event.path("build").path("GitBranch").asText());
    assertTrue(event.path("config").path("MAX_DISTANCE").isNumber());
    assertEquals(7.5, event.path("config").path("MAX_DISTANCE").asDouble());
    assertTrue(event.path("config").path("ENABLED").isBoolean());
    assertTrue(event.path("fieldLayout").path("sha256").asText().matches("[0-9a-f]{64}"));
  }

  @Test
  void robotArtifactCarriesBuildIdentityAndResolvedDependencyVersions() throws IOException {
    try (var stream = getClass().getResourceAsStream("/vision-build.properties")) {
      assertNotNull(stream, "Robot artifact must carry build-time identity without runtime Git");
      Properties metadata = new Properties();
      metadata.load(stream);
      assertTrue(metadata.getProperty("GitSha", "").matches("[0-9a-f]{40}"));
      assertTrue(!metadata.getProperty("GitBranch", "").isBlank());
      assertTrue(metadata.getProperty("GitDirty", "").matches("true|false"));
      assertTrue(!metadata.getProperty("BuildTimeUtc", "").isBlank());
      assertTrue(
          !metadata.getProperty("Dependency/edu.wpi.first.wpilibj/wpilibj-java", "").isBlank());
      assertTrue(!metadata.getProperty("Dependency/com.github.jonahsnider/doglog", "").isBlank());
    }
  }

  @Test
  void missingBuildResourceCannotClaimACleanKnownRevision() {
    var metadata = VisionRunMetadata.loadBuildMetadata(null);
    assertEquals("MISSING", metadata.get("ResourceStatus"));
    assertEquals("unknown", metadata.get("GitSha"));
    assertEquals("unknown", metadata.get("GitDirty"));
  }

  @Test
  void layoutIdentityIgnoresTagOrderingButDetectsGeometryAndOriginChanges() {
    var first = new AprilTag(1, new Pose3d(1, 2, 3, Rotation3d.kZero));
    var second = new AprilTag(2, new Pose3d(4, 5, 6, Rotation3d.kZero));
    var original = new AprilTagFieldLayout(List.of(first, second), 16, 8);
    var reordered = new AprilTagFieldLayout(List.of(second, first), 16, 8);
    String originalIdentity = VisionRunMetadata.sha256(VisionRunMetadata.describeLayout(original));
    assertEquals(
        originalIdentity, VisionRunMetadata.sha256(VisionRunMetadata.describeLayout(reordered)));

    var movedTag = new AprilTag(1, new Pose3d(1.001, 2, 3, Rotation3d.kZero));
    var changed = new AprilTagFieldLayout(List.of(movedTag, second), 16, 8);
    assertNotEquals(
        originalIdentity, VisionRunMetadata.sha256(VisionRunMetadata.describeLayout(changed)));
    reordered.setOrigin(new Pose3d(1, 0, 0, Rotation3d.kZero));
    assertNotEquals(
        originalIdentity, VisionRunMetadata.sha256(VisionRunMetadata.describeLayout(reordered)));
  }

  @Test
  void cameraTransformMetadataPreservesSubMillimeterPrecision() {
    var transform = new Transform3d(new Translation3d(.123456789, 0, 0), Rotation3d.kZero);
    assertEquals(
        "[0.123456789, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]", VisionRunMetadata.formatValue(transform));
  }
}
