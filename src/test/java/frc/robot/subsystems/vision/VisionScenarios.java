// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

/**
 * Factory for synthetic {@link PoseObservation}s used by the vision-stability tests.
 *
 * <p>Scenarios are split into two groups:
 *
 * <ul>
 *   <li><b>Accepted-by-design</b>: {@link #goodMultiTag} (and the realistic, flip-vulnerable {@link
 *       #flippedSingleTag} that intentionally passes the gates).
 *   <li><b>Rejected-by-design</b>: {@link #outOfBounds}, {@link #highZ}, {@link #tooFar}, {@link
 *       #singleTagHighAmbiguity} — each trips exactly one rejection gate.
 * </ul>
 */
final class VisionScenarios {

  private VisionScenarios() {}

  /** Two tags, low ambiguity, ~2 m away, in bounds. Tight std-dev; always accepted. */
  static PoseObservation goodMultiTag(double x, double y, double headingRad, double timestamp) {
    return obs(timestamp, pose(x, y, 0.0, headingRad), 0.08, 2, 2.0, new int[] {1, 2});
  }

  /**
   * Single tag at moderate range with just-under-threshold ambiguity. Passes every gate, but its
   * distance-scaled std-dev is large, so a flipped pose here must NOT teleport the fused estimate.
   * This is the realistic teleport vector the lean filter must survive via std-dev weighting.
   */
  static PoseObservation flippedSingleTag(double x, double y, double headingRad, double timestamp) {
    return obs(timestamp, pose(x, y, 0.0, headingRad), 0.28, 1, 4.0, new int[] {7});
  }

  /** Single tag with high ambiguity — tripped by the single-tag ambiguity gate. */
  static PoseObservation singleTagHighAmbiguity(double x, double y, double timestamp) {
    return obs(timestamp, pose(x, y, 0.0, 0.0), 0.60, 1, 3.0, new int[] {3});
  }

  /** Pose far outside the field — tripped by the field-bounds gate. */
  static PoseObservation outOfBounds(double timestamp) {
    return obs(timestamp, pose(-3.0, -3.0, 0.0, 0.0), 0.10, 2, 2.0, new int[] {1, 2});
  }

  /** Unrealistic Z height — tripped by the Z gate. */
  static PoseObservation highZ(double x, double y, double timestamp) {
    return obs(timestamp, pose(x, y, 1.5, 0.0), 0.10, 2, 2.0, new int[] {1, 2});
  }

  /** Average tag distance beyond the max — tripped by the distance gate. */
  static PoseObservation tooFar(double x, double y, double timestamp) {
    return obs(timestamp, pose(x, y, 0.0, 0.0), 0.10, 2, 7.0, new int[] {1, 2});
  }

  private static Pose3d pose(double x, double y, double z, double yawRad) {
    return new Pose3d(x, y, z, new Rotation3d(0.0, 0.0, yawRad));
  }

  private static PoseObservation obs(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double avgDistance,
      int[] tagIDs) {
    return new PoseObservation(
        timestamp,
        pose,
        ambiguity,
        tagCount,
        avgDistance,
        PoseObservationType.PHOTONVISION,
        tagIDs);
  }
}
