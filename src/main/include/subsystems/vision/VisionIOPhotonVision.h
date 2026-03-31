#pragma once

#include <functional>
#include <string>

#include <frc/geometry/Transform3d.h>
#include <frc/Timer.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>

#include "subsystems/vision/VisionIO.h"

namespace vision {

class VisionIOPhotonVision : public VisionIO {
 public:
  using SwerveDriveState = ctre::phoenix6::swerve::SwerveDrivetrain<
      ctre::phoenix6::hardware::TalonFX,
      ctre::phoenix6::hardware::TalonFX,
      ctre::phoenix6::hardware::CANcoder>::SwerveDriveState;

  VisionIOPhotonVision(std::string const& name,
                       frc::Transform3d const& robotToCamera,
                       std::function<SwerveDriveState()> swerveDriveStateSupplier);

  void ResetHeadingData();
  std::string GetCameraName() override;
  void UpdateInputs(VisionIOInputs& inputs) override;

 protected:
  photon::PhotonCamera m_camera;
  frc::Transform3d m_robotToCamera;
  photon::PhotonPoseEstimator m_poseEstimator;
  std::function<SwerveDriveState()> m_swerveDriveStateSupplier;
};

}  // namespace vision
