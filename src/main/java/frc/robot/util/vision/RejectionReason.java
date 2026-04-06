package frc.robot.util.vision;

// Enum for rejection reasons
public enum RejectionReason {
  NO_TAGS("No tags detected"),
  HIGH_AMBIGUITY("Ambiguity too high"),
  INVALID_Z_COORDINATE("Z coordinate out of bounds"),
  OUT_OF_FIELD_BOUNDS("Position outside field boundaries"),
  TOO_FAR_FROM_TAGS("Too far from AprilTags"),
  LARGE_POSE_CHANGE("Large change from previous pose"),
  COPLANAR_TOO_FAR("Coplanar multi-tag too far — flip risk"),
  COPLANAR_HIGH_AMBIGUITY("Coplanar multi-tag ambiguity too high — flip risk"),
  SINGLE_TAG_TOO_FAR("Single tag too far — flip risk"),
  SINGLE_TAG_HIGH_AMBIGUITY("Single tag ambiguity too high — flip risk"),
  INTER_FRAME_JUMP("Pose jumped too far between frames — likely flip");

  private final String description;

  RejectionReason(String description) {
    this.description = description;
  }

  public String getDescription() {
    return description;
  }
}
