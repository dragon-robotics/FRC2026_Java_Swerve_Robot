# Collecting vision logs for diagnosis and tuning

The vision subsystem sends each accepted camera observation to CTRE separately. The estimator
continues to combine vision with wheel odometry and gyro. Camera trust is logged as **standard
deviations**, in meters for X/Y and radians for heading, not covariance entries.

## Collect a run

1. Start logging before startup localization. Include stationary time, a short straight drive,
   turns, and the situation where the problem appears. Note any independently measured robot poses.
2. Keep the original `.wpilog`. On the robot, WPILib normally uses `/u/logs` with a writable USB drive,
   otherwise `/home/lvuser/logs`. Desktop simulation uses the checkout's `logs` directory.
3. Supply the log, approximate problem timestamp, what the robot physically did, and any changes
   to camera calibration/mounting/field layout. Run metadata identifies the built revision,
   whether the checkout was dirty, dependency versions and the robot's actual loaded layout.

DriverStation/control state and normal drivetrain/performance telemetry are captured in the same
file. Camera images and coprocessor calibration files are not embedded. Keep those separately when
investigating a mounting, calibration or detection issue.

## Export a compact file

From this checkout, with the normal Java/Gradle dependencies available:

```powershell
.\gradlew.bat exportVisionLog --offline --no-daemon "-PvisionLog=C:\logs\run.wpilog" "-PvisionExport=C:\logs\run.jsonl"
```

The output must be a new path. Export preserves the input and rejects malformed/truncated logs
instead of publishing a partial output. Supported format is WPILOG 1.x below 2 GiB. Each JSONL line
contains `entry`, `logTimestampMicros`, and `data`. This compact file can be shared for analysis;
keep the original WPILOG for full drivetrain, DS and performance context. Export is not full robot
replay and does not rerun PhotonVision's image solver.

## Atomic event records (schema version 1)

DogLog prefixes these entries with `/Robot/` in the WPILOG.

| Entry | Contents |
| --- | --- |
| `Vision/Run/Event` | Build identity, runtime mode, uncertainty/gating configuration, layout geometry/hash, declared camera transforms |
| `Vision/<camera>/Observation` | One pose observation and its final acceptance/rejection decision |
| `Vision/<camera>/Frame` | Frames with no targets, no pose, excessive range, or invalid solve metadata |
| `Swerve/PoseReset/Event` | Pose before/requested/after reset, reset reason, timestamp and robot mode |

An observation includes capture time in FPGA seconds, the converted CTRE timestamp, processing
time/frame age, camera/frame sequence, actual contributing tag IDs, source/solver, raw pose,
capture-time predicted pose, XY innovation, calculated uncertainty, uncertainty actually supplied,
enabled/auto state, startup camera/count, robot speeds/tilt and estimator pose around submission.
Rejected observations have `suppliedStdDevs: null`; no-history references and nonfinite numbers are
also null. `rawPose` is `[x,y,z,roll,pitch,yaw]`; 2D poses are `[x,y,heading]`; angles are radians
unless the field explicitly says Degrees. `speeds` is `[vx,vy,omega]` in m/s, m/s, rad/s.

CTRE updates asynchronously: `fusedPoseAfter` is an immediate readback, not proof that the native
estimator has already incorporated that measurement. Use the continuous drivetrain `Pose` and
`Timestamp` streams to inspect resulting motion/corrections.

`Vision/ActiveCameras`, per-camera `Connected` and `Configuration/*` topics identify actual active
cameras and settings. Raw/accepted/rejected pose-array topics remain available for field plots.

The Elastic **Vision** tab shows the actual selected solver separately for the front, right and
left cameras. Hybrid chooses independently for each camera/frame, so there is no single global
active solver. Live `/Robot/Vision/<camera>/CurrentStrategy` values include
`MULTI_TAG_PNP_ON_COPROCESSOR`, `CONSTRAINED_SOLVEPNP`, `PNP_DISTANCE_TRIG_SOLVE`, and
`LOWEST_AMBIGUITY`. The adjacent `StrategyStatus` reports `ACCEPTED` or the rejection/failure
reason; a displayed solver does not by itself mean its pose was fused. Both string topics are
published to NetworkTables even when ordinary NT logging is disabled.

The latest frame's strategy is held between camera updates for at most the 0.5-second frame-age
limit. A newer no-target/no-pose frame clears it to `NONE`; disconnects and stale or invalid
timestamps also show `NONE` with the corresponding status. These summaries supplement the
complete per-frame observation records above, which retain every strategy change in a batch.

DogLog reports its queue depth/capacity and `MAX_QUEUED_LOGS` on overload. Check the original log
for queue saturation or event-sequence gaps before treating a missing record as no measurement.

## Current decisions to interpret

- Valid observations from all cameras are submitted in capture-time order within each robot loop.
  Repeated/out-of-order timestamps are rejected independently for each camera; one camera does not
  suppress another. Frames older than 0.5 seconds, future frames, and missing-history frames are rejected.
- Startup solving is coprocessor MultiTag, then lowest ambiguity. After five-pose qualification,
  the default HYBRID order uses tag facing and motion, with capture-time heading for constrained/trig.
  Constrained PnP uses a coprocessor seed when available; rotation guards remain 0.5/1.0 rad/s.
  See [strategy order and test evidence](vision-strategy-test-audit.md).
- Translation uncertainty starts at `max(0.02 m, 0.10 * distance² / tagCount * cameraFactor * singleTagFactor)`;
  the single-tag factor is 5, X/Y use equal values, and heading sigma is `1e9` radians. These are
  conservative starting coefficients, not hardware-calibrated error guarantees. Stationary simulation
  showed larger errors for some tag geometries; distance and tag count alone do not describe every
  solve's accuracy. Aiming does not change trust.
- Camera factors resolve by configured camera name, so disabling B does not give its factor to L.
- Startup requires five stable accepted coprocessor MultiTag poses from the same camera. The first
  qualifying camera supplies its fifth pose; exact capture-time ties follow configured camera order.
  Subsequent initial-disabled refreshes use that camera. Enabling once permanently closes automatic
  reseeding for the program run. Manual reseeds remain explicit and logged.

Tune against actual measured poses where possible. Innovation measures disagreement with the
estimator; it is not independent ground truth. Begin with camera calibration/transforms/layout,
then examine distance-dependent residuals, camera biases, frame age, rejection causes and reset
events before changing uncertainty factors. Recheck timing/log volume on the roboRIO.
