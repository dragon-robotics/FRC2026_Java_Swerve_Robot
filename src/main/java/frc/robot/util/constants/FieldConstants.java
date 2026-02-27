package frc.robot.util.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;

public final class FieldConstants {
  // The different layouts of the AprilTags on the field
  public static final AprilTagFieldLayout APTAG_FIELD_LAYOUT;

  static {
    Path defaultPath =
        Path.of(
            Filesystem.getDeployDirectory().getPath(),
            "apriltags",
            "welded",
            "2026-rebuilt-welded.json");
    AprilTagFieldLayout defaultLayout = null;
    try {
      defaultLayout = new AprilTagFieldLayout(defaultPath);
    } catch (IOException e) {
      System.err.println("!!! CRITICAL: Failed to load default AprilTag field resource!");
      DriverStation.reportError(
          "CRITICAL: Failed to load default AprilTag field resource: " + e.getMessage(), true);
      defaultLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
    APTAG_FIELD_LAYOUT = defaultLayout;
  }

  public static final double FIELD_LENGTH = APTAG_FIELD_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH = APTAG_FIELD_LAYOUT.getFieldWidth();

  public static final class Hub {
    public static final double BLUE_HUB_CENTER_X =
        (APTAG_FIELD_LAYOUT.getTagPose(20).get().getX()
                + APTAG_FIELD_LAYOUT.getTagPose(26).get().getX())
            / 2;
    public static final edu.wpi.first.math.geometry.Translation2d BLUE_HUB_CENTER_POSE =
        new edu.wpi.first.math.geometry.Translation2d(
            BLUE_HUB_CENTER_X, APTAG_FIELD_LAYOUT.getTagPose(20).get().getY());
    public static final double RED_HUB_CENTER_X =
        (APTAG_FIELD_LAYOUT.getTagPose(4).get().getX()
                + APTAG_FIELD_LAYOUT.getTagPose(10).get().getX())
            / 2;
    public static final edu.wpi.first.math.geometry.Translation2d RED_HUB_CENTER_POSE =
        new edu.wpi.first.math.geometry.Translation2d(
            RED_HUB_CENTER_X, APTAG_FIELD_LAYOUT.getTagPose(4).get().getY());
  }
}
