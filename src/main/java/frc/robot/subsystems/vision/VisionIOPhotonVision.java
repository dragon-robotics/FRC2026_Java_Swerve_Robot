package frc.robot.subsystems.vision;

import static frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final PhotonPoseEstimator poseEstimator;

  protected final Supplier<SwerveDriveState> swerveDriveStateSupplier;

  // Static no-target constant to avoid allocation when no target is detected
  private static final TargetObservation NO_TARGET =
      new TargetObservation(new Rotation2d(), new Rotation2d());

  // Pre-allocated reusable collections — cleared each cycle instead of re-created
  private final Set<Short> tagIds = new HashSet<>(16);
  private final List<PoseObservation> poseObservations = new ArrayList<>(4);

  // Pre-allocated empty arrays for when there are no results
  private static final PoseObservation[] EMPTY_POSE_OBSERVATIONS = new PoseObservation[0];
  private static final int[] EMPTY_TAG_IDS = new int[0];

  /**
   * Creates a new VisionIOPV.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   * @param rotationSupplier The current heading of the robot.
   */
  public VisionIOPhotonVision(
      String name, Transform3d robotToCamera, Supplier<SwerveDriveState> swerveDriveStateSupplier) {

    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.swerveDriveStateSupplier = swerveDriveStateSupplier;
    poseEstimator = new PhotonPoseEstimator(APTAG_FIELD_LAYOUT, robotToCamera);

    // Reset heading data before pose initialization //
    poseEstimator.resetHeadingData(
        Timer.getFPGATimestamp(), swerveDriveStateSupplier.get().Pose.getRotation());
  }

  public void resetHeadingData() {
    poseEstimator.resetHeadingData(
        Timer.getFPGATimestamp(), swerveDriveStateSupplier.get().Pose.getRotation());
  }

  @Override
  public String getCameraName() {
    // Get the camera object
    return camera.getName();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.setConnected(camera.isConnected());
    inputs.setCameraName(camera.getName());

    // Update pose estimation heading data //
    poseEstimator.addHeadingData(
        Timer.getFPGATimestamp(), swerveDriveStateSupplier.get().Pose.getRotation());

    // Clear pre-allocated collections instead of creating new ones
    tagIds.clear();
    poseObservations.clear();

    for (var result : camera.getAllUnreadResults()) {
      // Use static constant when no target
      inputs.setLatestTargetObservation(NO_TARGET);
      if (!result.hasTargets()) {
        continue;
      }

      PhotonTrackedTarget bestTarget = result.getBestTarget();

      inputs.setLatestTargetObservation(
          new TargetObservation(
              Rotation2d.fromDegrees(bestTarget.getYaw()),
              Rotation2d.fromDegrees(bestTarget.getPitch())));

      // Replace ifPresent lambda with isPresent+get to avoid lambda capture allocation
      EstimatedRobotPose visionEst = poseEstimator.estimateCoprocMultiTagPose(result).orElse(null);

      if (visionEst != null) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        int targetCount = targets.size();

        // Single-pass computation
        double totalDistance = 0;
        double totalAmbiguity = 0;

        for (PhotonTrackedTarget target : targets) {
          totalDistance += target.bestCameraToTarget.getTranslation().getNorm();
          totalAmbiguity += target.getPoseAmbiguity();
          tagIds.add((short) target.getFiducialId());
        }

        poseObservations.add(
            new PoseObservation(
                visionEst.timestampSeconds,
                visionEst.estimatedPose,
                totalAmbiguity / targetCount,
                targetCount,
                totalDistance / targetCount,
                PoseObservationType.PHOTONVISION));
      }
    }

    // Save pose observations to inputs object — use cached empty array when possible
    inputs.setPoseObservations(
        poseObservations.isEmpty()
            ? EMPTY_POSE_OBSERVATIONS
            : poseObservations.toArray(EMPTY_POSE_OBSERVATIONS));

    // Save tag IDs to inputs objects — use cached empty array when possible
    if (tagIds.isEmpty()) {
      inputs.setTagIds(EMPTY_TAG_IDS);
    } else {
      int[] tagIdArray = new int[tagIds.size()];
      int idx = 0;
      for (short id : tagIds) {
        tagIdArray[idx++] = id;
      }
      inputs.setTagIds(tagIdArray);
    }
  }
}
