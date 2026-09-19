# Vision consensus experiment — 2026-09-19

Branch: `experiment/vision-consensus-latency`, based on `5fd4d4d`.

Experimental safeguards are implemented. Desktop simulation does **not** yet establish reliable localization across the full field or prove that the original real-robot autonomous incident is fixed. Camera-solve failures and intermittent loss of useful vision remain visible in the broader simulations.

## Implemented behavior

- Process at most the newest fresh frame per camera per loop. Discard older queued frames, repeats, and out-of-order frames before pose solving. An empty or failed newest frame does not cause an older-frame retry.
- Apply a 250 ms maximum observation age and 20 ms future tolerance at camera ingestion. Fusion additionally requires capture time to be no later than the latest native odometry sample.
- Invalidate historical sampling across pose, translation, rotation, and field-centric heading resets. Establish a conservative lower bound from the first observed native odometry update after startup/reset. Initial frames may be discarded until capture timestamps enter that window.
- Give each camera one vote. Compare each observation's XY residual against its capture-time odometry, so robot motion between exposures does not create false disagreement. Camera yaw does not participate in this translation comparison.
- Require a unique largest group whose members all agree pairwise within 0.45 m, comprising a strict majority of cameras with valid candidates that loop. Reject 1-vs-1, 2-vs-2, and overlapping/bridged ties. Work is bounded to at most four cameras and 15 subsets.
- Allow a lone camera while enabled only within 0.75 m of capture-time odometry; double its XY standard deviations. Keep the existing 2.5 m overall enabled innovation gate. Preserve disabled recovery and startup-only automatic hard reseeding.
- Fuse one original camera pose with its original capture timestamp converted to CTRE time. No averaging, synchronization wait, or extra multi-frame confirmation in steady-state fusion.
- Correct single-target fallback confidence: trig uses the best target; lowest-ambiguity uses its selected target. Coprocessor confidence uses its IDs-used list. Installed PhotonLib is characterized by a regression showing three visible targets in its raw single-target solve metadata.
- Use AprilTag local +X face normals for coplanarity. Preserve the previously requested slow-turning noncoplanar order: MultiTag → Constrained → Trig → Lowest Ambiguity.
- Log per-camera queue counts, selected age, processing and solver times, and consensus support, decision, innovation, and applied standard deviation.

Native history rationale: CTRE 26.3.0 keeps a 1.5 s odometry buffer, substantially longer than the experiment's 250 ms age limit. The application now also guards startup/reset boundaries and the newest sample rather than relying only on a nonempty optional. See [CTRE estimator source](https://api.ctr-electronics.com/phoenix6/stable/cpp/_swerve_drive_pose_estimator_8hpp_source.html).

## Validation design

The deterministic moving-estimator test uses independent ground truth, 2 m/s motion, 1% wheel-distance drift, and four cameras delayed by 20/60/100/140 ms. It exercises healthy observations, a backlog from one bad camera against two agreeing cameras, split votes, good/bad lone-camera observations, bridged clusters, and recovery. Production consensus and standard-deviation functions feed a WPILib estimator with the original capture measurements.

A separate native Phoenix/HAL integration test exercises the production observation filter and consumer handoff, pre-reset/stale/future history rejection, enabled solo limits, and disabled recovery after enable. Frame tests run the installed PhotonLib solvers and verify actual target metadata and queue behavior.

Existing PhotonVision/Phoenix simulation tests now report fresh timestamps separately from cached dashboard snapshots. Moving error is also measured against ground truth at capture time. Static tests fail when no fresh corrections occur; previously that case could pass without exercising localization.

## Results

Final full suite: **84 tests — 81 passed, 2 failed, 1 skipped**, Gradle exit 1. The failures are `VisionPoseStaticScenariosTest.rightFront` and `.leftBack`, both because no fresh correction occurred. `VisionPoseStaticTest` remains skipped for no accepted vision. New frame-policy, confidence, consensus, and native-history regressions pass. Targeted Java formatting check and `git diff --check` pass. This is deliberately not reported as an all-green suite.

Final-run measurements:

| Check | Result |
| --- | --- |
| Independent moving fault simulation | 200 useful updates accepted; all 90 designated fault decisions rejected |
| Maximum estimator error / excess jump | 0.022626 m / 0.003688 m |
| Consensus selector p50 / p95 / maximum | 4.5 / 11.6 / 103.5 microseconds, desktop |
| Native capture/reset/disabled-recovery integration | Passed |
| HYBRID left spin | 220 fresh updates / 250 cycles; 0.1333 m maximum capture-aligned error |
| HYBRID right spin | 224 / 250; 0.0847 m maximum capture-aligned error |
| HYBRID shuttle, two executions | 103 and 114 / 250; 0.3197 and 0.2990 m maximum capture-aligned error |
| HYBRID shuttle maximum excess odometry jump | 1.4187 and 1.4216 m; unresolved |
| HYBRID full scheduler p95 | 1.656–2.342 ms across these four desktop runs |
| Largest HYBRID scheduler duration | 22.551 ms; desktop simulation includes an over-20-ms spike |
| RightSideSafe smoke test | Passed its existing assertions, but recorded three large jumps, maximum 4.220 m |

The baseline shuttle runs already had 1.3840 and 1.3977 m excess jumps. This experiment does not demonstrate a shuttle-jump improvement. Baseline `right_front` and `left_back` also had zero vision cycles but passed; the stronger assertions now expose those coverage failures. These are single nondeterministic desktop runs, not controlled statistical performance comparisons.

Earlier exploratory runs already established these limitations:

- The deterministic test accepted 200 healthy/recovery decisions and rejected all 90 designated fault decisions. Maximum estimator error was 0.022626 m; maximum excess single-cycle jump was 0.003688 m.
- The full moving PhotonVision shuttle scenario still produced large jumps and intermittent low fresh-correction coverage. Separate executions varied substantially. These tests primarily report metrics and do not enforce the deterministic test's accuracy bounds.
- Static `left_back` produced zero fresh corrections in repeated runs. Diagnostic observations were rejected as out-of-bounds or beyond the distance limit. `right_front` varied between zero and one fresh correction in 250 measured cycles. A stationary estimate with no vision is not proof of working localization.
- Rear-camera construction is commented out in the existing SIM configuration. The deterministic fault test covers four cameras; the RobotContainer simulation uses three. This experiment preserves that configuration.
- The existing `RightSideSafeOdometryTest` permits up to five large jumps and does not bound the largest jump. Its simulated vision follows the estimator pose. Passing that smoke test does not prove independent-truth autonomous accuracy.

## Latency interpretation and remaining work

Queue work scans received results but solves at most one per camera per loop. Consensus adds no deliberate waiting. Desktop selector timing excludes camera solving and network latency; full scheduler timing includes other robot and simulation work. Neither is a roboRIO 2 measurement.

Next validation should replay a captured failing autonomous log and inspect camera calibration/extrinsics, strategy, actual tag IDs, age, innovation, and rejection reasons at each jump. Measure `Perf/Vision`, per-camera `SolverMs`, and robot-loop timing on the roboRIO 2 under representative camera load before promoting the experiment. Agreement cannot reject a shared camera/layout/calibration bias, and conservative rejection can leave the robot relying on wheel odometry.

## Reproduction and artifacts

Tests use the installed Gradle 8.11 distribution and existing dependency cache. `-x mirrorAutos` avoids regenerating unrelated autonomous assets. The final command is equivalent to:

```powershell
gradle --gradle-user-home C:/Users/dougd/.gradle -I build/vision-experiment/format.gradle test spotlessJavaCheck --offline --no-daemon --max-workers=1 -x mirrorAutos --console=plain
```

The test failures stopped that combined invocation before formatting, so `spotlessJavaCheck` was also run separately and passed (`build/vision-experiment/format-check.log`).

The temporary formatter init script restricts Java formatting to changed sources/tests. `VisionConstants.java` retains its pre-existing formatting; only the new constants and face-normal documentation are changed.

Local artifacts (generated, not checked in):

- `build/vision-experiment/baseline.log` and `baseline-artifacts/`: pre-change vision tests. The baseline included an outdated distance fixture at exactly the configured 7 m acceptance limit and a skipped static-auto test with no accepted vision. The fixture now uses 8 m.
- `build/vision-experiment/final-suite.log` and `build/reports/tests/test/index.html`: final full run.
- `build/vision-experiment/moving-fault-injection.csv` and `moving-fault-injection-summary.txt`: independent-truth fault simulation.
- `build/vision-stability/`: static and moving PhotonVision scenario CSVs.

No robot hardware was deployed or measured. Implementation is isolated on the experimental branch for review; the unresolved simulation results above remain part of the experiment.
