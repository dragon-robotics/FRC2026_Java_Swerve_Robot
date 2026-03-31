#pragma once

#include <photon/simulation/PhotonCameraSim.h>
#include <photon/simulation/SimCameraProperties.h>
#include <photon/simulation/VisionSystemSim.h>

#include "subsystems/vision/VisionIOPhotonVision.h"

namespace vision {

class VisionIOPhotonVisionSim : public VisionIOPhotonVision {
 public:
  VisionIOPhotonVisionSim(
      std::string const& name,
      frc::Transform3d const& robotToCamera,
      std::function<SwerveDriveState()> swerveDriveStateSupplier);

  std::string GetCameraName() override;
  void UpdateInputs(VisionIOInputs& inputs) override;

 private:
  photon::VisionSystemSim m_visionSim;
  std::unique_ptr<photon::PhotonCameraSim> m_cameraSim;
};

}  // namespace vision
