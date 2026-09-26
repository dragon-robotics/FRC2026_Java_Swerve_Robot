package frc.robot.util;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord.StartRecordData;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.HashMap;
import java.util.Map;

/** Exports atomic vision diagnostics from WPILOG to JSON lines. */
public final class VisionLogExport {
  private static final ObjectMapper JSON =
      new ObjectMapper().enable(DeserializationFeature.FAIL_ON_TRAILING_TOKENS);

  private VisionLogExport() {}

  public static void main(String[] arguments) {
    if (arguments.length != 2) {
      System.err.println("Usage: VisionLogExport <input.wpilog> <new-output.jsonl>");
      System.exit(2);
    }
    try {
      int count = export(Path.of(arguments[0]), Path.of(arguments[1]));
      System.out.println("Exported " + count + " vision/reset events to " + arguments[1]);
    } catch (IOException | IllegalArgumentException ex) {
      System.err.println("Vision log export failed: " + ex.getMessage());
      System.exit(2);
    }
  }

  static int export(Path input, Path output) throws IOException {
    Path destination = output.toAbsolutePath().normalize();
    if (Files.exists(destination)) {
      throw new IOException("Output already exists; choose a new file: " + destination);
    }
    try (var channel = FileChannel.open(input, StandardOpenOption.READ)) {
      long size = channel.size();
      if (size < 12 || size > Integer.MAX_VALUE) {
        throw new IOException("Invalid WPILOG size (supported range: 12 bytes through 2 GiB - 1)");
      }
      ByteBuffer buffer = channel.map(FileChannel.MapMode.READ_ONLY, 0, size);
      buffer.order(ByteOrder.LITTLE_ENDIAN);
      var reader = new DataLogReader(buffer.asReadOnlyBuffer());
      if (!reader.isValid() || (reader.getVersion() & 0xff00) != 0x0100) {
        throw new IOException("Invalid or unsupported WPILOG header");
      }
      validateFraming(buffer);

      Files.createDirectories(destination.getParent());
      Path temporary = Files.createTempFile(destination.getParent(), ".vision-export-", ".jsonl");
      int[] count = {0};
      try {
        Map<Integer, StartRecordData> entries = new HashMap<>();
        try (var writer = Files.newBufferedWriter(temporary, StandardCharsets.UTF_8)) {
          // WPILib's iterator can omit short final records. forEach visits every record; framing is
          // validated above because WPILib's reader otherwise silently accepts truncated tails.
          reader.forEach(
              record -> {
                try {
                  if (record.isStart()) {
                    var start = record.getStartData();
                    entries.put(start.entry, start);
                  } else if (record.isFinish()) {
                    entries.remove(record.getFinishEntry());
                  } else if (record.isSetMetadata()) {
                    record.getSetMetadataData(); // Validate this control payload too.
                  } else if (record.isControl()) {
                    throw new IOException("Malformed or unsupported WPILOG control record");
                  } else {
                    var entry = entries.get(record.getEntry());
                    if (entry == null) {
                      throw new IOException(
                          "WPILOG data references undefined entry " + record.getEntry());
                    }
                    if (!isDiagnosticEntry(entry.name)) {
                      return;
                    }
                    if (!entry.type.equals("string")) {
                      throw new IOException("Expected JSON string entry: " + entry.name);
                    }
                    var data = JSON.readTree(record.getString());
                    if (data == null || !data.isObject()) {
                      throw new IOException("Expected JSON object in " + entry.name);
                    }
                    var wrapper = JSON.createObjectNode();
                    wrapper.put("entry", entry.name);
                    wrapper.put("logTimestampMicros", record.getTimestamp());
                    wrapper.set("data", data);
                    writer.write(wrapper.toString());
                    writer.newLine();
                    count[0]++;
                  }
                } catch (IOException | RuntimeException ex) {
                  throw new UncheckedIOException(
                      new IOException(
                          "Malformed WPILOG event at timestamp "
                              + record.getTimestamp()
                              + " (entry "
                              + record.getEntry()
                              + "): "
                              + ex.getMessage(),
                          ex));
                }
              });
        } catch (UncheckedIOException ex) {
          throw ex.getCause();
        } catch (RuntimeException ex) {
          throw new IOException("Malformed WPILOG record: " + ex.getMessage(), ex);
        }
        // Never replace an existing destination or expose an incomplete export.
        Files.move(temporary, destination);
        return count[0];
      } finally {
        Files.deleteIfExists(temporary);
      }
    }
  }

  private static boolean isDiagnosticEntry(String name) {
    String key = name.startsWith("/Robot/") ? name.substring("/Robot/".length()) : name;
    return key.equals("Vision/Run/Event")
        || key.equals("Swerve/PoseReset/Event")
        || key.matches("Vision/[^/]+/(Observation|Frame)");
  }

  private static void validateFraming(ByteBuffer buffer) throws IOException {
    long position = 12L + Integer.toUnsignedLong(buffer.getInt(8));
    if (position > buffer.limit()) {
      throw new IOException("Truncated WPILOG extra header");
    }
    while (position < buffer.limit()) {
      int header = Byte.toUnsignedInt(buffer.get((int) position));
      int entryLength = (header & 0x3) + 1;
      int sizeLength = ((header >> 2) & 0x3) + 1;
      int timestampLength = ((header >> 4) & 0x7) + 1;
      int headerLength = 1 + entryLength + sizeLength + timestampLength;
      if ((header & 0x80) != 0) {
        throw new IOException("Invalid WPILOG record header at byte " + position);
      }
      if (position + headerLength > buffer.limit()) {
        throw new IOException("Truncated WPILOG record header at byte " + position);
      }
      long payloadLength = 0;
      for (int index = 0; index < sizeLength; index++) {
        payloadLength |=
            (long) Byte.toUnsignedInt(buffer.get((int) position + 1 + entryLength + index))
                << (8 * index);
      }
      long next = position + headerLength + payloadLength;
      if (next > buffer.limit()) {
        throw new IOException("Truncated WPILOG record payload at byte " + position);
      }
      position = next;
    }
  }
}
