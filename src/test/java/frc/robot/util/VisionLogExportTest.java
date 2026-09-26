package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class VisionLogExportTest {
  private static final ObjectMapper JSON = new ObjectMapper();
  @TempDir Path directory;

  @Test
  void exportsEveryAtomicVisionAndResetEventAsNestedJson() throws IOException {
    var fixture = new Fixture();
    fixture.start(1, "/Robot/Vision/Run/Event");
    fixture.start(2, "/Robot/Vision/front/Observation");
    fixture.start(3, "/Robot/Vision/front/Frame");
    fixture.start(4, "/Robot/Swerve/PoseReset/Event");
    fixture.start(5, "/Robot/Unrelated");
    String metadata =
        "{\"schemaVersion\":1,\"event\":\"RUN_METADATA\",\"branch\":\"test/\\\"quoted\\\"\"}";
    fixture.record(1, 1000, metadata);
    fixture.record(2, 1010, "{\"sequence\":1,\"pose\":[1.0,2.0,0.0]}");
    fixture.record(2, 1020, "{\"sequence\":2,\"pose\":[1.1,2.0,0.0]}");
    fixture.record(3, 1030, "{\"reason\":\"NO_TARGETS\"}");
    fixture.record(4, 1040, "{\"event\":\"POSE_RESET\"}");
    fixture.record(5, 1050, "not json and deliberately unrelated");
    Path input = fixture.write(directory.resolve("run.wpilog"));
    Path output = directory.resolve("run.jsonl");

    assertEquals(5, VisionLogExport.export(input, output));
    var lines = Files.readAllLines(output);
    assertEquals(5, lines.size());
    assertEquals(JSON.readTree(metadata), JSON.readTree(lines.get(0)).get("data"));
    assertEquals(
        "/Robot/Vision/front/Observation", JSON.readTree(lines.get(1)).get("entry").asText());
    assertEquals(1010, JSON.readTree(lines.get(1)).get("logTimestampMicros").asLong());
    assertEquals(2, JSON.readTree(lines.get(2)).get("data").get("sequence").asInt());
    assertEquals("NO_TARGETS", JSON.readTree(lines.get(3)).get("data").get("reason").asText());
    assertTrue(
        Arrays.equals(fixture.bytes(), Files.readAllBytes(input)), "Original log is untouched");
  }

  @Test
  void exportsACompleteShortFinalRecord() throws IOException {
    var fixture = new Fixture();
    fixture.start(1, "/Robot/Vision/front/Frame");
    fixture.compactRecord(1, 10, "{}");
    Path input = fixture.write(directory.resolve("short-final.wpilog"));
    Path output = directory.resolve("short-final.jsonl");
    assertEquals(1, VisionLogExport.export(input, output));
    assertEquals(10, JSON.readTree(Files.readString(output)).get("logTimestampMicros").asLong());
  }

  @Test
  void truncatedLogFailsWithoutPublishingAPartialExport() throws IOException {
    var fixture = new Fixture();
    fixture.start(1, "/Robot/Vision/front/Observation");
    fixture.record(1, 1000, "{\"sequence\":1}");
    byte[] complete = fixture.bytes();
    Path input = directory.resolve("truncated.wpilog");
    Files.write(input, Arrays.copyOf(complete, complete.length - 1));
    Path output = directory.resolve("truncated.jsonl");
    var error = assertThrows(IOException.class, () -> VisionLogExport.export(input, output));
    assertTrue(error.getMessage().contains("Truncated"));
    assertFalse(Files.exists(output));
  }

  @Test
  void malformedEventAndExistingOutputAreRejected() throws IOException {
    var fixture = new Fixture();
    fixture.start(1, "/Robot/Vision/front/Observation");
    fixture.record(1, 1000, "{bad json}");
    Path input = fixture.write(directory.resolve("malformed.wpilog"));
    Path output = directory.resolve("malformed.jsonl");
    assertThrows(IOException.class, () -> VisionLogExport.export(input, output));
    assertFalse(Files.exists(output));
    Files.writeString(output, "keep existing output");
    assertThrows(IOException.class, () -> VisionLogExport.export(input, output));
    assertEquals("keep existing output", Files.readString(output));
    assertThrows(IOException.class, () -> VisionLogExport.export(input, input));
  }

  /** Small WPILOG 1.0 fixture with real record headers and start-control payloads. */
  private static final class Fixture {
    private final ByteArrayOutputStream bytes = new ByteArrayOutputStream();
    private final DataOutputStream output = new DataOutputStream(bytes);

    Fixture() throws IOException {
      output.write("WPILOG".getBytes(StandardCharsets.US_ASCII));
      output.writeShort(0x0001); // little-endian version 0x0100
      output.writeInt(0); // no extra header
    }

    void start(int entry, String name) throws IOException {
      var controlBytes = new ByteArrayOutputStream();
      var control = new DataOutputStream(controlBytes);
      control.writeByte(0);
      control.writeInt(Integer.reverseBytes(entry));
      string(control, name);
      string(control, "string");
      string(control, "{}");
      record(0, 0, controlBytes.toByteArray());
    }

    void record(int entry, long timestamp, String value) throws IOException {
      record(entry, timestamp, value.getBytes(StandardCharsets.UTF_8));
    }

    void compactRecord(int entry, int timestamp, String value) throws IOException {
      byte[] payload = value.getBytes(StandardCharsets.UTF_8);
      output.writeByte(0); // 1-byte entry, size, and timestamp
      output.writeByte(entry);
      output.writeByte(payload.length);
      output.writeByte(timestamp);
      output.write(payload);
    }

    private void record(int entry, long timestamp, byte[] payload) throws IOException {
      output.writeByte(0x7c); // 1-byte entry, 4-byte payload length, 8-byte timestamp
      output.writeByte(entry);
      output.writeInt(Integer.reverseBytes(payload.length));
      output.writeLong(Long.reverseBytes(timestamp));
      output.write(payload);
    }

    private static void string(DataOutputStream target, String value) throws IOException {
      byte[] encoded = value.getBytes(StandardCharsets.UTF_8);
      target.writeInt(Integer.reverseBytes(encoded.length));
      target.write(encoded);
    }

    byte[] bytes() {
      return bytes.toByteArray();
    }

    Path write(Path path) throws IOException {
      return Files.write(path, bytes());
    }
  }
}
