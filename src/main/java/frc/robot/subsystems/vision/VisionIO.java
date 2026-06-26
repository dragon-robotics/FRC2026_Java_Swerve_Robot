package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Arrays;
import lombok.Getter;
import lombok.Setter;

/**
 * Hardware abstraction for robot vision cameras.
 *
 * <p>Implementations update {@link VisionIOInputs} with raw target angles and zero or more
 * timestamped robot-pose observations. The subsystem owns all filtering, weighting, and drivetrain
 * pose-estimator handoff.
 */
public interface VisionIO {

  /** Mutable camera input snapshot populated once per robot loop. */
  class VisionIOInputs {
    @Getter @Setter private String cameraName = "";
    @Getter @Setter private boolean connected = false;

    @Getter @Setter
    private TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());

    @Getter @Setter private PoseObservation[] poseObservations = new PoseObservation[0];
    @Getter @Setter private int[] tagIds = new int[0];

    /** Copies all fields from another input snapshot for thread-safe handoff. */
    public void copyFrom(VisionIOInputs other) {
      this.cameraName = other.cameraName;
      this.connected = other.connected;
      this.latestTargetObservation = other.latestTargetObservation;
      this.poseObservations = Arrays.copyOf(other.poseObservations, other.poseObservations.length);
      this.tagIds = Arrays.copyOf(other.tagIds, other.tagIds.length);
    }
  }

  /** Angle to the best visible target, where tx is yaw and ty is pitch. */
  record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /**
   * Robot pose estimate produced by one camera frame.
   *
   * @param timestamp FPGA timestamp in seconds for the camera frame
   * @param pose estimated robot pose in the field coordinate frame
   * @param ambiguity average PhotonVision pose ambiguity for the tags used
   * @param tagCount number of AprilTags used to estimate the pose
   * @param averageTagDistance distance-confidence value in meters consumed by acceptance gating
   * @param type solver/source classification for downstream trust decisions
   * @param tagIDs fiducial IDs used by the accepted pose solver
   */
  record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type,
      int[] tagIDs) {}

  /** Vision pose-estimation source used for filtering and standard-deviation selection. */
  enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION,
    PHOTONVISION_MULTITAG_COPROCESSOR
  }

  /** Refreshes camera inputs. Default implementation is for disconnected/fake cameras. */
  default void updateInputs(VisionIOInputs inputs) {}

  /** Returns the camera name used for alerts and telemetry. */
  default String getCameraName() {
    return "";
  }
}
