package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.APTAG_FIELD_LAYOUT;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
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

  // In constructor or as static finals
  private static final double YAW_THRESHOLD =
      VisionConstants.CAMERA_FOV_HORIZONTAL_DEGREES / 2.0 * 0.8;
  private static final double PITCH_THRESHOLD =
      VisionConstants.CAMERA_FOV_VERTICAL_DEGREES / 2.0 * 0.8;

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

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      inputs.setLatestTargetObservation(new TargetObservation(new Rotation2d(), new Rotation2d()));
      if (!result.hasTargets()) {
        continue;
      }

      PhotonTrackedTarget bestTarget = result.getBestTarget();

      inputs.setLatestTargetObservation(
          new TargetObservation(
              Rotation2d.fromDegrees(bestTarget.getYaw()),
              Rotation2d.fromDegrees(bestTarget.getPitch())));

      // Add pose observation based on how many tags we see
      Optional<EstimatedRobotPose> visionEst = poseEstimator.estimateCoprocMultiTagPose(result);

      visionEst.ifPresent(
          estimator -> {
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
                    estimator.timestampSeconds,
                    estimator.estimatedPose,
                    totalAmbiguity / targetCount,
                    targetCount,
                    totalDistance / targetCount,
                    PoseObservationType.PHOTONVISION));
          });
    }
    // Save pose observations to inputs object
    inputs.setPoseObservations(poseObservations.toArray(new PoseObservation[0]));

    // Save tag IDs to inputs objects
    int[] tagIdArray = new int[tagIds.size()];
    int idx = 0;
    for (short id : tagIds) {
      tagIdArray[idx++] = id;
    }
    inputs.setTagIds(tagIdArray);
  }
}
