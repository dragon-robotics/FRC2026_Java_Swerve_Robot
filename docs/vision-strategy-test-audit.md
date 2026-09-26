# Vision strategy revision and test audit — 2026-09-26

The controlled comparison supports a guarded HYBRID default after startup, with coprocessor
MultiTag used to seed constrained PnP whenever available. Keep the per-camera CTRE fusion,
uncertainty coefficients, rejection gates, and five accepted stable coprocessor MultiTag poses
from one camera for startup. Solver selection and estimator fusion are separate decisions.

## Production order

C = coprocessor MultiTag; K = constrained PnP; T = distance trig; L = lowest ambiguity.
The first solver that returns a pose wins; the subsystem then validates that observation.
A downstream rejection does not retry the other solvers on that frame.

| Condition | Attempt order |
| --- | --- |
| Before five-pose startup qualification | C → L |
| At least two visible tags, similarly facing, absolute rotation ≤0.5 rad/s | K → C → T → L |
| At least two visible tags, differently facing, absolute rotation ≤0.5 rad/s | C → K → T → L |
| One tag, absolute rotation ≤0.5 rad/s, translation ≤0.5 m/s | T → K → C → L |
| One tag, absolute rotation ≤0.5 rad/s, translation >0.5 m/s | T → C → K → L |
| At least two tags, absolute rotation >0.5 rad/s | C → T → L |
| One tag, absolute rotation >0.5 rad/s | T → C → L |

K additionally requires capture-time heading and camera calibration; it is skipped above
0.5 rad/s. Its seed order is C, then L, then capture-time odometry. T requires capture-time
heading and is skipped above 1.0 rad/s. Both reject nonfinite angular rates. Rotation limits
apply to both signs. These are existing conservative cutoffs, not newly calibrated optima.
Tag facing uses the tag's local +X normal (within 15 degrees), not its local +Z axis.

`vision.photon.strategyMode=STANDARD` selects C → L for comparisons.
`vision.photon.strategyOrder=...` overrides the post-initialization order. Neither property
bypasses startup's coprocessor-first requirement. Camera configuration and each frame's actual
solver remain logged. The run metadata now includes the HYBRID default constant.

## Why the old tests were insufficient

The former dynamic bakeoff reused the latest accepted snapshot across loops, compared a
capture-time observation with current truth, and often ranked a named primary solver while
its rate guard actually ran L. Startup could also override the requested solver. Several
assertions checked only summary counts. Its generated recommendations could recommend K
at high rotation, contrary to K's guard, and could pick a winner when all candidates failed.
Those rankings and generated recommendations have been removed.

The replacement replay generates each noisy frame once, then feeds that exact frame through
all five configurations: C-first, K-first, T-first, explicit HYBRID, and production default.
It runs 21 scenarios × 3 active cameras × 2 fixed seeds × 80 frames = 10,080 unique frames.
The scenarios include all eight established trench poses, both trenches spinning at 0.25,
0.5, 0.75 and 4.712 rad/s, a shuttle while spinning, and ±2-degree heading bias at both trenches.

Each record is compared with independent truth at capture, before estimator weighting.
The CSV records per-camera and combined frame/target/solve/quality-accept counts, actual solver
counts, rejection counts, raw RMSE/max for all solved poses, and RMSE/p95/max after static
quality gates. **Quality-accepted here does not mean submitted to CTRE:** replay deliberately
isolates solvers and the static gates; native timestamp/history/tilt/innovation/fusion paths
have separate integration tests. No uncertainty tuning can make these raw error metrics improve.

Regression gates require useful fresh observations, at least 80% of C-first coverage, bounded
RMSE and worst accepted errors in slow motion, actual constrained execution where intended,
and exact C-first equivalence above the trig rate cutoff. Heading-bias tolerance includes the
geometric range-times-angle error term, rather than expecting ideal-gyro accuracy with a biased gyro.

## Measured replay results

These are post-quality-gate XY RMSE, in centimeters, across F/R/L. The disabled B camera stays
disabled. The left-back view has no accepted default observations; zero coverage is explicitly
asserted and reported, never interpreted as zero localization error.

| Scenario | C-first RMSE (cm) | Selected RMSE (cm) | Selected p95 (cm) | C-first / selected accepted frames |
| --- | ---: | ---: | ---: | ---: |
| left_static_-90 | 6.84 | 0.21 | 0.43 | 160 / 160 |
| left_static_0 | 3.99 | 0.13 | 0.23 | 160 / 160 |
| left_static_90 | — | — | — | 0 / 0 |
| left_static_180 | 3.89 | 0.12 | 0.21 | 160 / 160 |
| left_spin_0.25 | 5.11 | 0.82 | 1.94 | 231 / 231 |
| left_spin_0.5 | 4.18 | 0.75 | 1.67 | 196 / 196 |
| left_spin_0.75 | 4.25 | 4.11 | 7.63 | 139 / 139 |
| left_spin_4.71238898038469 | 4.53 | 4.53 | 10.05 | 143 / 143 |
| right_static_-90 | 28.81 | 0.82 | 1.72 | 119 / 160 |
| right_static_0 | 4.37 | 0.14 | 0.24 | 160 / 160 |
| right_static_90 | 14.12 | 0.63 | 1.49 | 310 / 320 |
| right_static_180 | 26.58 | 1.00 | 2.24 | 282 / 320 |
| right_spin_0.25 | 11.79 | 1.22 | 2.93 | 210 / 212 |
| right_spin_0.5 | 17.18 | 0.89 | 2.10 | 237 / 247 |
| right_spin_0.75 | 14.87 | 14.82 | 36.97 | 220 / 220 |
| right_spin_4.71238898038469 | 13.40 | 13.40 | 31.21 | 181 / 181 |
| left_bias_-2.0 | 5.11 | 20.14 | 29.46 | 231 / 231 |
| right_bias_-2.0 | 11.79 | 14.43 | 23.61 | 210 / 212 |
| left_bias_2.0 | 5.11 | 19.43 | 28.37 | 231 / 231 |
| right_bias_2.0 | 11.79 | 14.57 | 23.93 | 210 / 212 |
| shuttle_spin | 3.47 | 3.47 | 5.36 | 222 / 222 |

The first corrected comparison failed the simplified C → L default. Restoring the former
hybrid policy then failed the heading-bias regression: the left-trench −2° case had 72.5 cm
RMSE with the old single-tag seed. Giving K the coprocessor seed first reduced that to 20.1 cm.
The remaining 14–20 cm errors under ±2° bias show the cost of inaccurate heading; hybrid does
not outperform C-first under every sensor fault. Without heading bias, slow/static selected
RMSE stays below 4 cm in this fixture. Fast rotation retains the C-first result.

The native stationary suite also checks fused XY error ≤0.35 m, heading error ≤2°, and
per-cycle jump ≤0.15 m. It preserves startup behavior, unlike the post-startup solver replay.
Its raw-noise sequence and native thread timing are not deterministic; exact numbers vary.

## Complete test inventory and disposition

| Test / helper | Action and reason |
| --- | --- |
| `VisionStrategyReplayTest` | Add deterministic same-frame solver comparison, explicit actual-solver coverage, all trench/motion cases, heading bias, positive/negative constrained rate boundaries. |
| `VisionDynamicStrategyBakeoffTest` | Remove; replacement eliminates duplicated runs, snapshot reuse, primary-name mislabeling, and weak count-only assertions. |
| `VisionStrategyComparisonSupport` | Remove; no remaining caller, and inferred hybrid recommendations were unsupported. |
| `VisionIOPhotonVisionMetadataTest` | Extend with real default single-tag behavior, missing heading/calibration fallback, ±trig boundary and unsafe rates, startup/restart override, empty-batch clearing. Use production batch processing instead of reflection. Retain actual contributing-tag and failed-frame checks. |
| `VisionPoseStaticScenariosTest` | Retain all eight native CTRE scenarios; count fresh latest snapshots only, add fused-position bound, fail initialization errors, preserve independent truth and stationary controls. This is not an all-camera coverage metric. |
| `VisionPoseStaticTest` | Remove redundant self-referential auto test that skipped when its vision never appeared. Its field pose remains in the eight-scenario suite. |
| `RightSideSafeOdometryTest` | Replace with `RightSideSafeSimulationSmokeTest`: assert auto completion, finite poses, and vision activity. Preserve jump CSV/diagnostics, remove the unsupported claim that several multi-meter jumps represent expected path-boundary resets. Runtime/initialization errors now fail instead of skipping. |
| `VisionFilterStabilityTest` | Retain deterministic noise/adversarial/uncertainty/copy and distinct cross-camera tests. Remove two arithmetic startup duplicates covered by the actual startup integration. Correct the claim that its WPILib estimator is the native CTRE estimator. |
| `VisionDistanceConfidencePolicyTest` | Retain average-versus-max calculation; replace enum-shape assertion with missing-sample rejection and exact 7 m boundary coverage. |
| `VisionStartupReseedTest` | Retain accepted-only, same-camera five-pose qualification, exact fifth snapshot, instability, expiration/recovery, duplicates, all-camera forwarding and diagnostic checks. Add both signs of pitch/roll rejection and five-pose recovery after leveling. |
| `VisionDisabledAutoReseedLifecycleTest` | Retain startup, first-enable latch, reconnect and new-program behavior. |
| `VisionScenarios` | Retain valid-ID and intentionally invalid measurement fixtures; too-far fixture is actually beyond the 7 m gate. |
| `SwerveVisionHistoryTest` | Retain native history endpoint, reset invalidation, telemetry callback and manual-heading-reset tests. |
| `VisionRunMetadataTest` | Retain build/config types, missing-build handling, field-layout identity and precise transforms. |
| `VisionLogExportTest` | Retain complete-event export, short final record, truncation and malformed/overwrite protection. |
| `VisionConsensusPerformanceTest` | Keep removed as part of the earlier fusion change: winner-only consensus is no longer the production contract. |

`visionStabilityTest` now includes native history and log/metadata tests as well as the vision
package. Both Gradle test entry points forward strategy mode/order. Removed the obsolete
`vision.applyCoplanarPenalty` forwarding. The normal full `test` task still includes this coverage.

## Verification result

Full suite: **103 tests, 0 failures, 0 errors, 0 skipped**.
Vision/history/log coverage accounts for 70 of those tests. The replay CSV contains
420 scenario/policy/camera summary rows. Targeted Java formatting and `git diff --check` pass.

The controlled solver replay retains its established 800×600, 72° camera fixture. The native
camera simulation was subsequently updated to 1280×800, 70° diagonal FOV, 40 FPS, and 20 ms
average latency (5 ms standard deviation). The full suite is rechecked with those settings;
solver replay and native integration results should not be treated as identical camera models.

## Reproduce and remaining limitations

```powershell
.\gradlew.bat test --offline --no-daemon --max-workers=1 --rerun-tasks -x mirrorAutos
.\gradlew.bat visionStabilityTest --offline --no-daemon --max-workers=1 -x mirrorAutos
```

If another simulator process holds `halsim_gui.dll`, add `-x extractReleaseNative` only when
the same dependencies have already been extracted. The final validation used that workaround
after the normal full-run command failed during extraction, before tests started.

Artifacts: `build/vision-strategy-revisit/solver-comparison.csv`,
`build/vision-strategy-revisit/full-validation.log`, `build/vision-stability/static-*.csv`,
`build/vision-stability/right-side-safe.csv`, and the normal JUnit reports.

The auto smoke fixture feeds estimator pose back into simulated cameras. It still shows
multi-meter jumps (3.69 m in the focused run); this revision does not establish their cause or
fix them. It is explicitly excluded from solver/localization evidence. The simulator also
uses matching camera/layout geometry and omits real occlusion, motion blur, calibration bias,
and realistic gyro timing faults. The sampled rotation rates exercise the guards; the cutoffs are not optimized here.
Replay real trench logs and measure heading/extrinsics on the robot before treating the policy
or the unchanged uncertainty model as hardware-validated.
