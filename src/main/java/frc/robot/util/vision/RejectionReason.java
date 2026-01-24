package frc.robot.util.vision;

// Enum for rejection reasons
public enum RejectionReason {
  NO_TAGS("No tags detected"),
  HIGH_AMBIGUITY("Ambiguity too high"),
  INVALID_Z_COORDINATE("Z coordinate out of bounds"),
  OUT_OF_FIELD_BOUNDS("Position outside field boundaries"),
  TOO_FAR_FROM_TAGS("Too far from AprilTags"),
  LARGE_POSE_CHANGE("Large change from previous pose");

  private final String description;

  RejectionReason(String description) {
    this.description = description;
  }

  public String getDescription() {
    return description;
  }
}
