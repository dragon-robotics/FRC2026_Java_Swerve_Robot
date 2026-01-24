package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;
import lombok.Setter;

public interface VisionIO {

  public static class VisionIOInputs {
    @Getter @Setter private String cameraName = "";
    @Getter @Setter private boolean connected = false;

    @Getter @Setter
    private TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());

    @Getter @Setter private PoseObservation[] poseObservations = new PoseObservation[0];
    @Getter @Setter private int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp, // seconds since of the pose observation
      Pose3d pose, // pose of the robot in the camera frame
      double ambiguity, // ambiguity of the pose estimate
      int tagCount, // number of tags used to estimate the pose
      double averageTagDistance, // average distance to the tags used to estimate the pose
      PoseObservationType type) {}

  public enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default String getCameraName() {
    return "";
  }
}
