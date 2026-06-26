package frc.robot.subsystems.vision;

import static frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** PhotonVision IO backed by PhotonLib simulation for deterministic vision tests. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static final String VISION_SIM_NAME = "main";
  private static final int CAMERA_RESOLUTION_WIDTH_PIXELS = 800;
  private static final int CAMERA_RESOLUTION_HEIGHT_PIXELS = 600;
  private static final double CAMERA_DIAGONAL_FOV_DEGREES = 72.0;
  private static final double CAMERA_AVERAGE_LATENCY_MS = 10.0;
  private static final double CAMERA_LATENCY_STD_DEV_MS = 5.0;
  private static final int CAMERA_FPS = 60;

  private final VisionSystemSim visionSim;
  private final PhotonCameraSim cameraSim;
  private final Supplier<Pose2d> poseSupplier;

  /**
   * Creates a simulated PhotonVision camera.
   *
   * @param name PhotonVision camera name
   * @param robotToCamera transform from robot frame to camera frame
   * @param poseSupplier current robot pose supplier in the field coordinate frame
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    visionSim = new VisionSystemSim(VISION_SIM_NAME);
    visionSim.addAprilTags(APTAG_FIELD_LAYOUT);

    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(
        CAMERA_RESOLUTION_WIDTH_PIXELS,
        CAMERA_RESOLUTION_HEIGHT_PIXELS,
        Rotation2d.fromDegrees(CAMERA_DIAGONAL_FOV_DEGREES));
    cameraProperties.setCalibError(0.38, 0.1);
    cameraProperties.setFPS(CAMERA_FPS);
    cameraProperties.setAvgLatencyMs(CAMERA_AVERAGE_LATENCY_MS);
    cameraProperties.setLatencyStdDevMs(CAMERA_LATENCY_STD_DEV_MS);

    cameraSim = new PhotonCameraSim(camera, cameraProperties, APTAG_FIELD_LAYOUT);

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public String getCameraName() {
    return camera.getName();
  }

  /** Advances the simulated vision system using the supplied robot pose. */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
