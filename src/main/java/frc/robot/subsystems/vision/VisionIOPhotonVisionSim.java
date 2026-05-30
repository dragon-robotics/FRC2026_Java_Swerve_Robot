package frc.robot.subsystems.vision;

import static frc.robot.util.constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

  private final VisionSystemSim visionSim;
  private final PhotonCameraSim cameraSim;
  private final Supplier<Pose2d> poseSupplier;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name          The name of the camera.
   * @param robotToCamera Transform from robot to camera.
   * @param poseSupplier  Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(APTAG_FIELD_LAYOUT);

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(800, 600, Rotation2d.fromDegrees(72));
    cameraProperties.setCalibError(0.38, 0.1);
    cameraProperties.setFPS(60);
    cameraProperties.setAvgLatencyMs(10);
    cameraProperties.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, cameraProperties, APTAG_FIELD_LAYOUT);

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public String getCameraName() {
    // Get the camera object
    return camera.getName();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}