#pragma once

#include "subsystems/vision/VisionIO.h"
#include "util/vision/RejectionReason.h"

#include <string>
#include <variant>

namespace vision {

struct AcceptedPose {
  PoseObservation poseObservation;
};

struct RejectedPose {
  PoseObservation poseObservation;
  RejectionReason reason;
  std::string details;
};

using PoseValidationResult = std::variant<AcceptedPose, RejectedPose>;

}  // namespace vision
