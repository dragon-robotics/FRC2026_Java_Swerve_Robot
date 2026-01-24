package frc.robot.util.vision;

import frc.robot.subsystems.vision.VisionIO.PoseObservation;

// Record for accepted poses
public record AcceptedPose(PoseObservation poseObservation) implements PoseValidationResult {}
