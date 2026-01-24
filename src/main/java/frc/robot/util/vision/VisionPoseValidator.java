package frc.robot.util.vision;

import static frc.robot.Constants.FieldConstants.APTAG_FIELD_LAYOUT;
import static frc.robot.Constants.VisionConstants.MAX_AMBIGUITY;
import static frc.robot.Constants.VisionConstants.MAX_Z_ERROR;

import frc.robot.subsystems.vision.VisionIO.PoseObservation;

public class VisionPoseValidator {
  private static final double MAX_TAG_DISTANCE = 8.0;

  // private static final double MAX_POSE_CHANGE = 3.0;

  // private Pose2d m_lastAcceptedPose = null;

  public PoseValidationResult validatePose(PoseObservation observation) {
    var pose = observation.pose();

    // Check for no tags
    if (observation.tagCount() == 0) {
      return new RejectedPose(
          observation, RejectionReason.NO_TAGS, "Tag count: " + observation.tagCount());
    }

    // Check ambiguity for single tag
    if (observation.tagCount() == 1 && observation.ambiguity() > MAX_AMBIGUITY) {
      return new RejectedPose(
          observation,
          RejectionReason.HIGH_AMBIGUITY,
          "Ambiguity: %.3f > %.3f".formatted(observation.ambiguity(), MAX_AMBIGUITY));
    }

    // Check Z coordinate
    if (Math.abs(pose.getZ()) > MAX_Z_ERROR) {
      return new RejectedPose(
          observation,
          RejectionReason.INVALID_Z_COORDINATE,
          "Z: %.3f > %.3f".formatted(Math.abs(pose.getZ()), MAX_Z_ERROR));
    }

    // Check field boundaries
    var pose2d = pose.toPose2d();
    if (pose2d.getX() < 0.0
        || pose2d.getX() > APTAG_FIELD_LAYOUT.getFieldLength()
        || pose2d.getY() < 0.0
        || pose2d.getY() > APTAG_FIELD_LAYOUT.getFieldWidth()) {
      return new RejectedPose(
          observation,
          RejectionReason.OUT_OF_FIELD_BOUNDS,
          "Position: (%.2f, %.2f)".formatted(pose2d.getX(), pose2d.getY()));
    }

    // Check distance from tags
    if (observation.averageTagDistance() > MAX_TAG_DISTANCE) {
      return new RejectedPose(
          observation,
          RejectionReason.TOO_FAR_FROM_TAGS,
          "Distance: %.2f > %.2f".formatted(observation.averageTagDistance(), MAX_TAG_DISTANCE));
    }

    // // Check large pose change
    // if (m_lastAcceptedPose != null) {
    // double poseChange = m_lastAcceptedPose.getTranslation()
    // .getDistance(pose2d.getTranslation());
    // if (poseChange > MAX_POSE_CHANGE) {
    // return new RejectedPose(observation, RejectionReason.LARGE_POSE_CHANGE,
    // "Change: %.2f > %.2f".formatted(poseChange, MAX_POSE_CHANGE));
    // }
    // }

    // Pose accepted - update last accepted pose
    // m_lastAcceptedPose = pose2d;
    return new AcceptedPose(observation);
  }
}
