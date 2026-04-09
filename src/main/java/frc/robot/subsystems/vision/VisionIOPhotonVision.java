package frc.robot.subsystems.vision;

import static frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT;
import static frc.robot.util.constants.VisionConstants.MAX_TAG_DISTANCE;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final PhotonPoseEstimator poseEstimator;

  private static final TargetObservation NO_TARGET =
      new TargetObservation(new Rotation2d(), new Rotation2d());

  // Pre-allocated reusable collections — avoids GC pressure on RoboRIO v1
  private final Set<Short> tagIds = new HashSet<>(16);
  private final List<PoseObservation> poseObservations = new ArrayList<>(4);

  private static final PoseObservation[] EMPTY_POSE_OBSERVATIONS = new PoseObservation[0];
  private static final int[] EMPTY_TAG_IDS = new int[0];

  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.poseEstimator = new PhotonPoseEstimator(APTAG_FIELD_LAYOUT, robotToCamera);
  }

  @Override
  public String getCameraName() {
    return camera.getName();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.setConnected(camera.isConnected());
    inputs.setCameraName(camera.getName());

    tagIds.clear();
    poseObservations.clear();

    var allResults = camera.getAllUnreadResults();
    if (allResults.isEmpty()) {
      inputs.setPoseObservations(EMPTY_POSE_OBSERVATIONS);
      inputs.setTagIds(EMPTY_TAG_IDS);
      inputs.setLatestTargetObservation(NO_TARGET);
      return;
    }

    // Only process the latest result — avoids CPU spikes from backlog on v1
    var result = allResults.get(allResults.size() - 1);

    if (result.hasTargets()) {
      PhotonTrackedTarget best = result.getBestTarget();
      inputs.setLatestTargetObservation(
          new TargetObservation(
              Rotation2d.fromDegrees(best.getYaw()), Rotation2d.fromDegrees(best.getPitch())));

      // Early distance filter: skip expensive PnP solver when all tags are too far
      boolean allTooFar = true;
      for (var t : result.getTargets()) {
        if (t.bestCameraToTarget.getTranslation().getNorm() <= MAX_TAG_DISTANCE) {
          allTooFar = false;
          break;
        }
      }
      if (allTooFar) {
        inputs.setPoseObservations(EMPTY_POSE_OBSERVATIONS);
        inputs.setTagIds(EMPTY_TAG_IDS);
        return;
      }

      // Try coprocessor multi-tag first (runs on camera, no RIO CPU cost), fall back
      // to lowest ambiguity
      EstimatedRobotPose visionEst =
          poseEstimator
              .estimateCoprocMultiTagPose(result)
              .or(() -> poseEstimator.estimateLowestAmbiguityPose(result))
              .orElse(null);
      if (visionEst != null) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        int count = targets.size();
        double totalDistance = 0;
        double totalAmbiguity = 0;
        int[] observedTagIDs = new int[count];

        for (int i = 0; i < count; i++) {
          PhotonTrackedTarget t = targets.get(i);
          totalDistance += t.bestCameraToTarget.getTranslation().getNorm();
          totalAmbiguity += t.getPoseAmbiguity();
          tagIds.add((short) t.getFiducialId());
          observedTagIDs[i] = t.getFiducialId();
        }

        poseObservations.add(
            new PoseObservation(
                visionEst.timestampSeconds,
                visionEst.estimatedPose,
                totalAmbiguity / count,
                count,
                totalDistance / count,
                PoseObservationType.PHOTONVISION,
                observedTagIDs));
      }
    } else {
      inputs.setLatestTargetObservation(NO_TARGET);
    }

    inputs.setPoseObservations(
        poseObservations.isEmpty()
            ? EMPTY_POSE_OBSERVATIONS
            : poseObservations.toArray(EMPTY_POSE_OBSERVATIONS));

    if (tagIds.isEmpty()) {
      inputs.setTagIds(EMPTY_TAG_IDS);
    } else {
      int[] arr = new int[tagIds.size()];
      int i = 0;
      for (short id : tagIds) arr[i++] = id;
      inputs.setTagIds(arr);
    }
  }
}
