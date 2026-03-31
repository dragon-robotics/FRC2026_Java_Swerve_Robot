#include "util/vision/VisionPoseValidator.h"

#include <cmath>
#include <fmt/format.h>

#include "Constants.h"

using namespace vision;

PoseValidationResult VisionPoseValidator::ValidatePose(PoseObservation const& observation) {
  auto pose = observation.pose;

  if (observation.tagCount == 0) {
    return {false, observation, RejectionReason::NO_TAGS,
            fmt::format("Tag count: {}", observation.tagCount)};
  }

  if (observation.tagCount == 1 && observation.ambiguity > VisionConstants::MAX_AMBIGUITY) {
    return {false, observation, RejectionReason::HIGH_AMBIGUITY,
            fmt::format("Ambiguity: {:.3f} > {:.3f}", observation.ambiguity,
                        VisionConstants::MAX_AMBIGUITY)};
  }

  if (std::abs(pose.Z().value()) > VisionConstants::MAX_Z_ERROR) {
    return {false, observation, RejectionReason::INVALID_Z_COORDINATE,
            fmt::format("Z: {:.3f} > {:.3f}", std::abs(pose.Z().value()),
                        VisionConstants::MAX_Z_ERROR)};
  }

  auto pose2d = pose.ToPose2d();
  auto layout = FieldConstants::GetAprilTagFieldLayout();
  if (pose2d.X().value() < 0.0 ||
      pose2d.X().value() > layout.GetFieldLength().value() ||
      pose2d.Y().value() < 0.0 ||
      pose2d.Y().value() > layout.GetFieldWidth().value()) {
    return {false, observation, RejectionReason::OUT_OF_FIELD_BOUNDS,
            fmt::format("Position: ({:.2f}, {:.2f})", pose2d.X().value(), pose2d.Y().value())};
  }

  if (observation.averageTagDistance > kMaxTagDistance) {
    return {false, observation, RejectionReason::TOO_FAR_FROM_TAGS,
            fmt::format("Distance: {:.2f} > {:.2f}", observation.averageTagDistance,
                        kMaxTagDistance)};
  }

  m_lastAcceptedPose = pose2d;
  m_hasLastPose = true;
  return {true, observation, {}, {}};
}
