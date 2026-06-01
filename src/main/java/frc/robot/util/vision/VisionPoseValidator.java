package frc.robot.util.vision;

import static frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT;
import static frc.robot.util.constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.Optional;

public class VisionPoseValidator {

  /** Last accepted vision pose — baseline for flip detection. */
  private Pose2d m_lastAcceptedPose = null;

  /** Timestamp of the last accepted vision pose. */
  private double m_lastAcceptedTimestamp = -1.0;

  // ──────────────────────────────────────────────────────────────────────────
  // Coplanar Detection
  // ──────────────────────────────────────────────────────────────────────────

  /**
   * Checks whether all tags in an observation lie on the same plane (same hub
   * face). When all tags
   * are coplanar, the multi-tag PnP solver has the same rotational ambiguity as
   * single-tag — the
   * planar geometry admits two valid solutions differing by 180° around the plane
   * normal.
   *
   * <p>
   * Detection: compare the Z-axis (outward normal) of each tag's field pose. If
   * all normals are
   * within
   * {@link frc.robot.util.constants.VisionConstants#COPLANAR_ANGLE_THRESHOLD_DEG}
   * of each
   * other, the tags are coplanar.
   *
   * @param tagIDs array of visible tag IDs
   * @return true if all tags are coplanar (flip-vulnerable)
   */
  private boolean areTagsCoplanar(int[] tagIDs) {
    if (tagIDs == null || tagIDs.length <= 1) {
      return true; // Single tag is trivially coplanar
    }

    Optional<edu.wpi.first.math.geometry.Pose3d> firstPoseOpt = APTAG_FIELD_LAYOUT.getTagPose(tagIDs[0]);
    if (firstPoseOpt.isEmpty())
      return true; // Unknown tag — treat as vulnerable

    Rotation3d referenceRotation = firstPoseOpt.get().getRotation();
    double thresholdRad = Math.toRadians(COPLANAR_ANGLE_THRESHOLD_DEG);

    for (int i = 1; i < tagIDs.length; i++) {
      Optional<edu.wpi.first.math.geometry.Pose3d> tagPoseOpt = APTAG_FIELD_LAYOUT.getTagPose(tagIDs[i]);
      if (tagPoseOpt.isEmpty())
        continue;

      // Compute relative rotation between this tag and the reference.
      // Coplanar tags face the same direction → relative rotation ≈ identity.
      Rotation3d relative = referenceRotation.unaryMinus().plus(tagPoseOpt.get().getRotation());
      double angle = relative.getAngle(); // total rotation angle in radians

      if (angle > thresholdRad) {
        return false; // Tags on different planes — true multi-tag ✅
      }
    }

    return true; // All tags face the same way — coplanar, flip-vulnerable
  }

  /**
   * Returns true if the observation is "effectively single-tag" for flip
   * rejection and standard
   * deviation purposes. True when:
   *
   * <ul>
   * <li>tagCount == 1 (actually single tag), OR
   * <li>tagCount &gt;= 2 but all tags are coplanar (same hub face)
   * </ul>
   *
   * <p>
   * Exposed as public so {@code VisionSubsystem.calculateStandardDeviations} can
   * use it to
   * distrust rotation from coplanar multi-tag observations.
   */
  public boolean isEffectivelySingleTag(PoseObservation observation) {
    if (observation.tagCount() <= 1)
      return true;
    return areTagsCoplanar(observation.tagIDs());
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Validation
  // ──────────────────────────────────────────────────────────────────────────

  public PoseValidationResult validatePose(PoseObservation observation) {
    var pose = observation.pose();

    // ── No tags ──────────────────────────────────────────────────────────
    if (observation.tagCount() == 0) {
      return new RejectedPose(
          observation, RejectionReason.NO_TAGS, "Tag count: " + observation.tagCount());
    }

    boolean effectivelySingleTag = isEffectivelySingleTag(observation);
    boolean actuallySingleTag = observation.tagCount() == 1;

    // ── Layer 1: Distance gate ───────────────────────────────────────────
    // Single-tag and coplanar multi-tag are both flip-vulnerable at distance.
    // Coplanar gets a slightly more generous limit since multiple coplanar
    // tags improve translational (but not rotational) accuracy.
    if (effectivelySingleTag) {
      double maxDist = actuallySingleTag ? SINGLE_TAG_MAX_DISTANCE_METERS : COPLANAR_MAX_DISTANCE_METERS;

      if (observation.averageTagDistance() > maxDist) {
        RejectionReason reason = actuallySingleTag
            ? RejectionReason.SINGLE_TAG_TOO_FAR
            : RejectionReason.COPLANAR_TOO_FAR;
        return new RejectedPose(
            observation,
            reason,
            "Distance: %.2f > %.2f".formatted(observation.averageTagDistance(), maxDist));
      }
    }

    // // ── Layer 2: Tighter ambiguity for flip-vulnerable observations ──────
    // // Single-tag gets the tightest threshold (highest flip risk).
    // // Coplanar multi-tag gets a more relaxed threshold — multiple tags
    // // improve translational accuracy, and the distance gate + inter-frame
    // // jump check provide additional flip protection.
    // if (effectivelySingleTag) {
    // double maxAmbiguity = actuallySingleTag ? SINGLE_TAG_MAX_AMBIGUITY :
    // COPLANAR_MAX_AMBIGUITY;

    // if (observation.ambiguity() > maxAmbiguity) {
    // RejectionReason reason = actuallySingleTag
    // ? RejectionReason.SINGLE_TAG_HIGH_AMBIGUITY
    // : RejectionReason.COPLANAR_HIGH_AMBIGUITY;
    // return new RejectedPose(
    // observation,
    // reason,
    // "Ambiguity: %.3f > %.3f".formatted(observation.ambiguity(), maxAmbiguity));
    // }
    // }

    // ── General ambiguity check (true multi-tag) ─────────────────────────
    if (!effectivelySingleTag && observation.ambiguity() > MAX_AMBIGUITY) {
      return new RejectedPose(
          observation,
          RejectionReason.HIGH_AMBIGUITY,
          "Ambiguity: %.3f > %.3f".formatted(observation.ambiguity(), MAX_AMBIGUITY));
    }

    // ── Z coordinate check ───────────────────────────────────────────────
    double absZ = Math.abs(pose.getZ());
    if (absZ > MAX_Z_ERROR) {
      return new RejectedPose(
          observation,
          RejectionReason.INVALID_Z_COORDINATE,
          "Z: %.3f > %.3f".formatted(absZ, MAX_Z_ERROR));
    }

    // ── Field boundaries ─────────────────────────────────────────────────
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

    // ── General distance check (true multi-tag uses original limit) ──────
    // effectivelySingleTag observations already passed their tighter gate above.
    if (!effectivelySingleTag && observation.averageTagDistance() > MAX_TAG_DISTANCE) {
      return new RejectedPose(
          observation,
          RejectionReason.TOO_FAR_FROM_TAGS,
          "Distance: %.2f > %.2f".formatted(observation.averageTagDistance(), MAX_TAG_DISTANCE));
    }

    // ── Layer 3: Timestamp-based flip detection ──────────────────────────
    // Compare vision displacement against what is PHYSICALLY POSSIBLE given
    // the elapsed time and the robot's maximum speed. This avoids any
    // circular dependency with odometry — it's a pure physics constraint.
    //
    // The allowed displacement is:
    // max = dt * MAX_ROBOT_SPEED_MPS * SPEED_TOLERANCE_MULTIPLIER
    // + BASE_JUMP_TOLERANCE_METERS
    //
    // A flip produces 2-5m displacement in a single 20ms frame:
    // max = 0.02 * 4.76 * 1.5 + 0.5 = 0.64m -- 3.2m flip REJECTED
    //
    // Legitimate gap (2s) + fast driving:
    // max = 2.0 * 4.76 * 1.5 + 0.5 = 14.78m -- 8m move ACCEPTED
    if (m_lastAcceptedPose != null && m_lastAcceptedTimestamp > 0.0) {
      double dt = observation.timestamp() - m_lastAcceptedTimestamp;

      // Clamp dt: avoid negatives from timestamp rollover and cap at a
      // reasonable maximum to prevent unbounded tolerance after very long
      // gaps (e.g. autonomous → teleop transition).
      dt = Math.max(0.0, Math.min(dt, MAX_FLIP_DETECTION_DT_SECONDS));

      double visionDelta = m_lastAcceptedPose.getTranslation().getDistance(pose2d.getTranslation());

      double maxAllowedDelta = dt * MAX_ROBOT_SPEED_MPS * SPEED_TOLERANCE_MULTIPLIER + BASE_JUMP_TOLERANCE_METERS;

      if (visionDelta > maxAllowedDelta) {
        return new RejectedPose(
            observation,
            RejectionReason.INTER_FRAME_JUMP,
            "dt: %.3fs, delta: %.2fm > max: %.2fm".formatted(dt, visionDelta, maxAllowedDelta));
      }
    }

    // ── Pose accepted ────────────────────────────────────────────────────
    // Update baselines ONLY after all checks pass — a rejected pose must
    // not contaminate the flip detection baseline.
    m_lastAcceptedPose = pose2d;
    m_lastAcceptedTimestamp = observation.timestamp();
    return new AcceptedPose(observation);
  }
}