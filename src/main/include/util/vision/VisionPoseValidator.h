#pragma once

#include <string>

#include "subsystems/vision/VisionIO.h"

namespace vision {

enum class RejectionReason {
  NO_TAGS,
  HIGH_AMBIGUITY,
  INVALID_Z_COORDINATE,
  OUT_OF_FIELD_BOUNDS,
  TOO_FAR_FROM_TAGS,
  LARGE_POSE_CHANGE
};

struct PoseValidationResult {
  bool accepted = false;
  PoseObservation poseObservation{};
  RejectionReason reason{};
  std::string details{};
};

class VisionPoseValidator {
 public:
  PoseValidationResult ValidatePose(PoseObservation const& observation);

 private:
  static constexpr double kMaxTagDistance = 8.0;
  static constexpr double kMaxPoseChange = 3.0;

  bool m_hasLastPose = false;
  frc::Pose2d m_lastAcceptedPose{};
};

}  // namespace vision
