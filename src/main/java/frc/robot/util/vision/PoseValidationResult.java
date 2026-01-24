package frc.robot.util.vision;

// Sealed interface for pose validation results
public sealed interface PoseValidationResult permits AcceptedPose, RejectedPose {}
