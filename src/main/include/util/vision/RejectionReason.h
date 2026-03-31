#pragma once

#include <string>

namespace vision {

enum class RejectionReason {
  NO_TAGS,
  HIGH_AMBIGUITY,
  INVALID_Z_COORDINATE,
  OUT_OF_FIELD_BOUNDS,
  TOO_FAR_FROM_TAGS,
  LARGE_POSE_CHANGE
};

inline std::string GetRejectionDescription(RejectionReason reason) {
  switch (reason) {
    case RejectionReason::NO_TAGS:
      return "No tags detected";
    case RejectionReason::HIGH_AMBIGUITY:
      return "Ambiguity too high";
    case RejectionReason::INVALID_Z_COORDINATE:
      return "Z coordinate out of bounds";
    case RejectionReason::OUT_OF_FIELD_BOUNDS:
      return "Position outside field boundaries";
    case RejectionReason::TOO_FAR_FROM_TAGS:
      return "Too far from AprilTags";
    case RejectionReason::LARGE_POSE_CHANGE:
      return "Large change from previous pose";
    default:
      return "Unknown";
  }
}

}  // namespace vision
