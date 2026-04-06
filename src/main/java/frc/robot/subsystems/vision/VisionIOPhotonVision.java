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
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

  // Disconnect tracking — prevents NT stderr spam and stale heading buffer growth
  private boolean wasConnected = false;
  private double disconnectedSinceTimestamp = -1.0;

  // After this many seconds disconnected, skip getAllUnreadResults() entirely
  private static final double DISCONNECT_POLL_TIMEOUT_SECONDS = 0.5;

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
    poseEstimator =
        new PhotonPoseEstimator(
            APTAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

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
    return camera.getName();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    boolean connected = camera.isConnected();
    inputs.setConnected(connected);
    inputs.setCameraName(camera.getName());

    double now = Timer.getFPGATimestamp();

    // Reconnect detected — reset heading buffer so stale angles are discarded
    if (connected && !wasConnected) {
      poseEstimator.resetHeadingData(now, swerveDriveStateSupplier.get().Pose.getRotation());
      disconnectedSinceTimestamp = -1.0;
    }

    // Disconnected — track duration; after timeout skip poll entirely (stops NT
    // stderr spam)
    if (!connected) {
      if (disconnectedSinceTimestamp < 0.0) {
        disconnectedSinceTimestamp = now;
      }
      wasConnected = false;
      if ((now - disconnectedSinceTimestamp) > DISCONNECT_POLL_TIMEOUT_SECONDS) {
        inputs.setPoseObservations(EMPTY_POSE_OBSERVATIONS);
        inputs.setTagIds(EMPTY_TAG_IDS);
        inputs.setLatestTargetObservation(NO_TARGET);
        return;
      }
    }

    wasConnected = connected;

    // Only feed heading data when connected — prevents buffer growth during
    // disconnect
    if (connected) {
      poseEstimator.addHeadingData(now, swerveDriveStateSupplier.get().Pose.getRotation());
    }

    // Clear pre-allocated collections instead of creating new ones
    tagIds.clear();
    poseObservations.clear();

    // Get all unread results — only process the latest one to prevent backlog
    // spikes
    var allResults = camera.getAllUnreadResults();
    if (allResults.isEmpty()) {
      inputs.setPoseObservations(EMPTY_POSE_OBSERVATIONS);
      inputs.setTagIds(EMPTY_TAG_IDS);
      return;
    }

    // Only process the LATEST result — discard all stale frames
    var result = allResults.get(allResults.size() - 1);

    inputs.setLatestTargetObservation(NO_TARGET);

    if (result.hasTargets()) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();

      inputs.setLatestTargetObservation(
          new TargetObservation(
              Rotation2d.fromDegrees(bestTarget.getYaw()),
              Rotation2d.fromDegrees(bestTarget.getPitch())));

      // Try coprocessor multi-tag first, fall back to standard estimator
      var coprocResult = poseEstimator.estimateCoprocMultiTagPose(result);
      EstimatedRobotPose visionEst;
      if (coprocResult.isPresent()) {
        visionEst = coprocResult.get();
      } else if (result.getTargets().size() >= 2) {
        visionEst = poseEstimator.update(result).orElse(null);
      } else {
        visionEst = null;
      }

      if (visionEst != null) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        int targetCount = targets.size();

        double totalDistance = 0;
        double totalAmbiguity = 0;
        int[] observedTagIDs = new int[targetCount];

        for (int i = 0; i < targetCount; i++) {
          PhotonTrackedTarget target = targets.get(i);
          totalDistance += target.bestCameraToTarget.getTranslation().getNorm();
          totalAmbiguity += target.getPoseAmbiguity();
          tagIds.add((short) target.getFiducialId());
          observedTagIDs[i] = target.getFiducialId();
        }

        poseObservations.add(
            new PoseObservation(
                visionEst.timestampSeconds,
                visionEst.estimatedPose,
                totalAmbiguity / targetCount,
                targetCount,
                totalDistance / targetCount,
                PoseObservationType.PHOTONVISION,
                observedTagIDs));
      }
    }

    inputs.setPoseObservations(
        poseObservations.isEmpty()
            ? EMPTY_POSE_OBSERVATIONS
            : poseObservations.toArray(EMPTY_POSE_OBSERVATIONS));

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
