#pragma once

#include <string>
#include <vector>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>

namespace vision {

struct TargetObservation {
  frc::Rotation2d tx{};
  frc::Rotation2d ty{};
};

enum class PoseObservationType {
  MEGATAG_1,
  MEGATAG_2,
  PHOTONVISION
};

struct PoseObservation {
  double timestamp = 0.0;
  frc::Pose3d pose{};
  double ambiguity = 0.0;
  int tagCount = 0;
  double averageTagDistance = 0.0;
  PoseObservationType type = PoseObservationType::PHOTONVISION;
};

struct VisionIOInputs {
  std::string cameraName{};
  bool connected = false;
  TargetObservation latestTargetObservation{};
  std::vector<PoseObservation> poseObservations{};
  std::vector<int> tagIds{};

  void CopyFrom(VisionIOInputs const& other) {
    cameraName = other.cameraName;
    connected = other.connected;
    latestTargetObservation = other.latestTargetObservation;
    poseObservations = other.poseObservations;
    tagIds = other.tagIds;
  }
};

class VisionIO {
 public:
  virtual ~VisionIO() = default;
  virtual void UpdateInputs(VisionIOInputs& inputs) {}
  virtual std::string GetCameraName() { return ""; }
};

}  // namespace vision
