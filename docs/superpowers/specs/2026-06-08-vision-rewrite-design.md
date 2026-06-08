# Vision "Back to Basics" Rewrite — Design

Date: 2026-06-08
Branch: `feature/vision-improvements`
Status: Implemented autonomously (user unavailable; review async)

## Context

The current vision stack is large and exhibits two field-visible failures:

1. **Swerve pose teleports**, making the robot undrivable.
2. **Low vision pose acceptance.**

References used to anchor the redesign:

- AdvantageKit vision template (`Mechanical-Advantage/AdvantageKit` @ `a86d21b`).
- Team 1678 `C2026-Public` PhotonVision pose-estimation strategy.

## Root cause

- **Teleporting:** `VisionSubsystem` hard-resets the drivetrain with `swerve.resetPose(visionPose)`
  in three paths (odometry init, `tryReseedFromVision`, `forceReseedFromVision`). A single flipped /
  bad single-tag estimate snaps the whole pose. Neither AdvantageKit nor 1678 hard-reset during
  operation — they feed `addVisionMeasurement` and let the Kalman filter blend with standard
  deviations.
- **Low acceptance:** `VisionPoseValidator` stacks aggressive, interacting gates (cross-camera
  consistency, coplanar yaw current + historical, physics-based inter-frame flip detection) plus a
  constrained-solvePnP fallback. Together they reject most observations.

## Goals

- Predictable, drivable pose estimation: vision **never** hard-resets the pose during normal
  operation; it only contributes weighted measurements.
- Trust translation, **ignore vision heading** (gyro is authoritative) — 1678 strategy.
- Tighter translation trust while actively aiming/aligning to score — 1678 strategy.
- Reject tags beyond ~5.5 m — 1678 strategy.
- Drastically smaller, single-file rejection logic that is easy to read and tune.

## Non-goals

- Re-tuning camera transforms (unchanged).
- Replacing the IO layer (it is already AdvantageKit-style and is reused).
- Changing the dashboard overlay contract (`AcceptedObservationSnapshot` API preserved).

## Decisions

1. **Full rewrite** of `VisionSubsystem`; reuse `VisionIO` / `VisionIOPhotonVision(Sim)`.
2. **No automatic pose resets.** Keep exactly one operator-triggered manual reseed
   (`forceReseedFromVision`) for recovery.
3. **Aiming signal** pushed from `Superstructure` via `vision.setAiming(boolean)`.
4. **Hybrid std-dev model:** distance-scaled translation (`baseline · dist² / tagCount`), heading
   std-dev fixed to a huge constant (ignored), and translation tightened by a multiplier while
   aiming.
5. **Delete** the `util/vision` validator classes; keep the dashboard overlay backed by the existing
   `AcceptedObservationSnapshot` accessor.

## New behavior

### Per-observation rejection (simple, documented)

Reject when any of:

- `tagCount == 0`
- `|pose.Z| > MAX_Z_ERROR`
- pose outside field bounds
- `tagCount == 1 && ambiguity > MAX_AMBIGUITY`
- `averageTagDistance > MAX_AVG_TAG_DISTANCE_METERS` (5.5 m)

Accepted observations are fed to the consumer (`swerve::addVisionMeasurement`) and recorded for the
dashboard snapshot.

### Standard deviations

```
factor       = averageTagDistance^2 / tagCount
linearStdDev = LINEAR_STDDEV_BASELINE * factor * cameraFactor * (aiming ? AIM_LINEAR_STDDEV_MULTIPLIER : 1.0)
angularStdDev = HEADING_STDDEV_IGNORE   // vision heading ignored; gyro is authoritative
```

### Public API (preserved for existing callers)

- `VisionSubsystem(CommandSwerveDrivetrain, VisionConsumer, VisionIO...)`
- `VisionConsumer`
- `AcceptedObservationSnapshot` + `getLatestAcceptedObservationSnapshot()` (dashboard)
- `forceReseedFromVision()` (operator manual recovery)
- **New:** `setAiming(boolean)`

### Removed

- `resetOdometryInitialized()`, `tryReseedFromVision()`, odometry-init gating, cross-camera /
  coplanar / flip validators, per-camera reseed bookkeeping.
- `util/vision/{VisionPoseValidator,PoseValidationResult,AcceptedPose,RejectedPose,RejectionReason}.java`
- Validator-only constants in `VisionConstants`.

## Files touched

- `subsystems/vision/VisionSubsystem.java` — rewritten lean.
- `util/constants/VisionConstants.java` — add lean constants, prune validator-only ones.
- `util/vision/*` — deleted.
- `subsystems/Superstructure.java` — `vision.setAiming(...)` from `periodic()`; keep reseed command.
- `Robot.java` — drop `resetOdometryInitialized()` call at teleop start (preserve unrelated RT-priority change).

## Risks & validation

- Risk: ignoring vision heading relies on a well-zeroed gyro — acceptable and standard (1678).
- Risk: removing flip rejection could let a rare flipped single-tag pose through, but high
  translation std-dev at distance + the 5.5 m gate + single-tag ambiguity gate bound its impact, and
  it can no longer hard-reset the pose.
- Validation: `./gradlew.bat spotlessApply` + `./gradlew.bat compileJava`. Field/sim behavior to be
  confirmed by the user.
