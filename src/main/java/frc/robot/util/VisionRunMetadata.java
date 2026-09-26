package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.GeneralConstants;
import frc.robot.util.constants.SwerveConstants;
import frc.robot.util.constants.VisionConstants;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Array;
import java.lang.reflect.Modifier;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HexFormat;
import java.util.Map;
import java.util.Properties;
import java.util.TreeMap;
import java.util.UUID;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/** Records the build and effective configuration needed to interpret one robot run. */
public final class VisionRunMetadata {
  public static final int SCHEMA_VERSION = 1;
  private static final String PREFIX = "Vision/Run/";

  private VisionRunMetadata() {}

  /** Called once during robot startup, after DogLog has been configured. */
  public static void log() {
    String runId = UUID.randomUUID().toString();
    String runtimeMode = GeneralConstants.CURRENT_MODE.name();
    DogLog.log(PREFIX + "SchemaVersion", SCHEMA_VERSION);
    DogLog.log(PREFIX + "RunId", runId);
    DogLog.log(PREFIX + "RuntimeMode", runtimeMode);
    var build =
        loadBuildMetadata(VisionRunMetadata.class.getResourceAsStream("/vision-build.properties"));
    build.forEach((key, value) -> DogLog.log(PREFIX + "Build/" + key, value));

    // Snapshot public constants once, including system-property overrides already resolved by the
    // constants class. This also captures new gates/weights without a second configuration list.
    Map<String, Object> config = new TreeMap<>();
    for (var field : VisionConstants.class.getFields()) {
      if (!Modifier.isStatic(field.getModifiers())) {
        continue;
      }
      try {
        Object value = field.get(null);
        DogLog.log(PREFIX + "Config/" + field.getName(), formatValue(value));
        config.put(field.getName(), structuredValue(value));
      } catch (IllegalAccessException ex) {
        DogLog.log(PREFIX + "Config/" + field.getName(), "unavailable: " + ex.getMessage());
        config.put(field.getName(), "unavailable: " + ex.getMessage());
      }
    }
    DogLog.log(PREFIX + "Config/ODOMETRY_STD", formatValue(SwerveConstants.ODOMETRY_STD));
    config.put("ODOMETRY_STD", structuredValue(SwerveConstants.ODOMETRY_STD));
    String layout = describeLayout(FieldConstants.APTAG_FIELD_LAYOUT);
    DogLog.log(PREFIX + "FieldLayout/Geometry", layout);
    DogLog.log(PREFIX + "FieldLayout/Sha256", sha256(layout));
    DogLog.log(
        PREFIX + "Event", createRunEvent(runId, runtimeMode, build, config, layout).toString());
  }

  static Map<String, String> loadBuildMetadata(InputStream stream) {
    Map<String, String> values = new TreeMap<>();
    values.put("GitSha", "unknown");
    values.put("GitBranch", "unknown");
    values.put("GitDirty", "unknown");
    if (stream == null) {
      values.put("ResourceStatus", "MISSING");
      return values;
    }
    try (stream) {
      Properties properties = new Properties();
      properties.load(stream);
      for (String key : properties.stringPropertyNames()) {
        values.put(key, properties.getProperty(key));
      }
      values.put("ResourceStatus", "LOADED");
    } catch (IOException | IllegalArgumentException ex) {
      values.put("ResourceStatus", "READ_ERROR: " + ex.getMessage());
    }
    return values;
  }

  static ObjectNode createRunEvent(
      String runId,
      String runtimeMode,
      Map<String, String> build,
      Map<String, Object> config,
      String layout) {
    var json = new ObjectMapper();
    var event = json.createObjectNode();
    event.put("schemaVersion", SCHEMA_VERSION);
    event.put("event", "RUN_METADATA");
    event.put("runId", runId);
    event.put("runtimeMode", runtimeMode);
    event.set("build", json.valueToTree(build));
    event.set("config", json.valueToTree(config));
    var fieldLayout = event.putObject("fieldLayout");
    fieldLayout.put("geometry", layout);
    fieldLayout.put("sha256", sha256(layout));
    return event;
  }

  /**
   * Canonical geometry identifies the layout actually loaded, including any fallback and origin.
   */
  static String describeLayout(AprilTagFieldLayout layout) {
    StringBuilder description =
        new StringBuilder("lengthMeters=")
            .append(layout.getFieldLength())
            .append(";widthMeters=")
            .append(layout.getFieldWidth())
            .append(";poseOrder=x,y,z,qw,qx,qy,qz;origin=")
            .append(describePose(layout.getOrigin()));
    layout.getTags().stream()
        .sorted(Comparator.comparingInt(tag -> tag.ID))
        .forEach(
            tag ->
                description
                    .append(";tag/")
                    .append(tag.ID)
                    .append('=')
                    .append(describePose(tag.pose)));
    return description.toString();
  }

  static String sha256(String value) {
    try {
      return HexFormat.of()
          .formatHex(
              MessageDigest.getInstance("SHA-256").digest(value.getBytes(StandardCharsets.UTF_8)));
    } catch (NoSuchAlgorithmException ex) {
      throw new IllegalStateException("Java runtime does not provide SHA-256", ex);
    }
  }

  static String formatValue(Object value) {
    if (value instanceof Transform3d transform) {
      return describePose(new Pose3d(transform.getTranslation(), transform.getRotation()));
    }
    if (value instanceof Matrix<?, ?> matrix) {
      return Arrays.toString(matrix.getData());
    }
    if (value != null && value.getClass().isArray()) {
      return IntStream.range(0, Array.getLength(value))
          .mapToObj(index -> formatValue(Array.get(value, index)))
          .collect(Collectors.joining(", ", "[", "]"));
    }
    return String.valueOf(value);
  }

  private static Object structuredValue(Object value) {
    if (value instanceof Transform3d transform) {
      return poseValues(new Pose3d(transform.getTranslation(), transform.getRotation()));
    }
    if (value instanceof Matrix<?, ?> matrix) {
      return matrix.getData().clone();
    }
    if (value != null && value.getClass().isArray()) {
      return IntStream.range(0, Array.getLength(value))
          .mapToObj(index -> structuredValue(Array.get(value, index)))
          .toList();
    }
    return value;
  }

  private static String describePose(Pose3d pose) {
    return Arrays.toString(poseValues(pose));
  }

  private static double[] poseValues(Pose3d pose) {
    var rotation = pose.getRotation().getQuaternion();
    return new double[] {
      pose.getX(),
      pose.getY(),
      pose.getZ(),
      rotation.getW(),
      rotation.getX(),
      rotation.getY(),
      rotation.getZ()
    };
  }
}
