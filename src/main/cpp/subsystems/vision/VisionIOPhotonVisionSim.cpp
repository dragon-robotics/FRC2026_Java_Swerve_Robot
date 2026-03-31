#include "subsystems/vision/VisionIOPhotonVisionSim.h"

#include "Constants.h"

using namespace vision;

VisionIOPhotonVisionSim::VisionIOPhotonVisionSim(
    std::string const& name,
    frc::Transform3d const& robotToCamera,
    std::function<SwerveDriveState()> swerveDriveStateSupplier)
    : VisionIOPhotonVision{name, robotToCamera, std::move(swerveDriveStateSupplier)},
      m_visionSim{"main"} {
  m_visionSim.AddAprilTags(FieldConstants::GetFieldLayout());

  photon::SimCameraProperties cameraProperties{};
  cameraProperties.SetCalibration(640, 480, frc::Rotation2d{72_deg});
  cameraProperties.SetCalibError(0.38, 0.1);
  cameraProperties.SetFPS(60_Hz);
  cameraProperties.SetAvgLatency(10_ms);
  cameraProperties.SetLatencyStdDev(5_ms);

  m_cameraSim = std::make_unique<photon::PhotonCameraSim>(
      &m_camera, cameraProperties, FieldConstants::GetFieldLayout());

  m_visionSim.AddCamera(m_cameraSim.get(), robotToCamera);
}

std::string VisionIOPhotonVisionSim::GetCameraName() {
  return m_camera.GetCameraName();
}

void VisionIOPhotonVisionSim::UpdateInputs(VisionIOInputs& inputs) {
  m_visionSim.Update(m_swerveDriveStateSupplier().Pose);
  VisionIOPhotonVision::UpdateInputs(inputs);
}
