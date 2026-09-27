# Vision strategy revision and test audit — 2026-09-26

## Constrained eligibility through 90 degrees/second — 2026-09-27

The production constrained-PnP limit is now `Math.PI / 2` rad/s, inclusive in both directions.
The Hybrid selector and the constrained solver guard share that limit. Trig retains its
independent 1 rad/s limit. Nonfinite rates make both heading-dependent solvers ineligible.

The drivetrain supplies heading and pose history to these solvers only after an absolute
field pose/rotation reset establishes alignment. Five-pose qualification while enabled does
not establish alignment. Independent coprocessor/lowest-ambiguity acquisition remains available;
startup still requires five accepted stable coprocessor MultiTag observations from one camera.

`VisionConstrainedMotionReplayTest` adds four-camera coverage of both reported positions and
both trench centers: 80 trajectories, 224 heading conditions, 76,800 generated frames and
430,080 actual IO evaluations. It covers both rotation signs, rates below/at/above 90 degrees/s,
translated acceleration/deceleration, +/-2-degree heading bias and +/-10/20 ms heading offsets.
The original three-camera fixtures remain separate. Aggregate, per-frame and paired comparison
CSVs are written under `build/vision-constrained-motion/`.

The sweep passed all 80 trajectories and 224 heading conditions. At exactly +/-90 degrees/s,
combining both directions and both noise seeds, accepted camera-measurement results were:

| Position | Maximum XY error | XY RMSE | Instants with an accepted camera / 480 |
| --- | ---: | ---: | ---: |
| Reported (4.72, 0.59) | 0.0375 m | 0.0061 m | 480 |
| Reported (6.996, 2.130) | 0.0372 m | 0.0057 m | 468 |
| Right trench (4.407, 0.650) | 0.0303 m | 0.0056 m | 480 |
| Left trench (4.407, 7.279) | 0.0288 m | 0.0060 m | 477 |

Across all constant-rate cases at or below the limit, maximum accepted XY error was 0.0663 m,
RMSE stayed below 0.0077 m, and sampled any-camera availability was at least 97.5%.
The translated acceleration/deceleration traces cross above the limit and resume constrained
solving afterward. Their eligible-rate maximum error stayed below 0.067 m, but faster portions
retained coprocessor-fallback errors up to 0.394 m. Above-limit fixed-rate cases executed no
constrained solves and matched the coprocessor/lowest-ambiguity reference exactly.

Within the 90 degrees/s limit, +/-2-degree heading bias produced accepted errors up to
0.339 m; +/-20 ms heading offsets produced up to 0.305 m at 90 degrees/s (1.8 degrees of heading
error). A +/-10 ms offset reached 0.156 m. These errors can pass the heading gate because the
solve and gate share the same faulty heading. Widening the rate limit does not solve calibration
or capture-time-heading faults.

`VisionStrategyLatencyTest` separately emulates 40 ms delayed frames with acceleration of
+/-2 rad/s squared. Both rotation directions cross above and below the threshold, and both
Hybrid and explicit constrained-first orders use processing-time angular rate for eligibility
while retaining capture-time heading and timestamps. The eight solver outcomes are exported
under `build/vision-strategy-latency/`.

These simulations compare camera measurements with independent analytic truth. The nominal
accuracy contract assumes correct field heading and capture timing. Injected heading faults
explicitly expose translation bias that can pass a gate using the same heading reference.
Image generation uses current camera geometry, two deterministic corner-noise seeds, and sampled
trajectories; it does not model exposure blur or real camera scheduling. These are measurement
errors before CTRE fusion. Native CTRE fusion/startup/history tests run separately in the suite.
The test heading is wrapped consistently with its 3D pose seed; native-history regressions
cover reset angles beyond +/-pi and a full turn to check the production provider's representation.
No test establishes hardware calibration, motion-blur tolerance, or roboRIO timing performance.

Final validation: all 146 tests across 21 classes passed, with zero failures, errors, or skips.
This includes the complete sweep, original replay/trench fixtures, delayed-frame crossings,
native CTRE fusion/history/recovery, and five-pose startup regressions. Scoped Spotless checks
passed for the twelve Java files changed by this work and the preceding quality-gate follow-up.

## Quality-gate follow-up — 2026-09-27

The initial same-frame quality follow-up applied the production 6 m two-tag coprocessor distance
gate and the 5-degree capture-time heading-consistency gate once field heading is aligned. Solver
order was unchanged at that stage. The historical results below describe the earlier revisions.

Two full-rotation scenarios at (6.996, 2.130) m and +/-0.74 rad/s use the current 1280x800,
70-degree fixture with all four cameras. The original 21 scenarios retain their 800x600/72-degree
three-camera fixture and fixed seeds. The regression requires the raw two-tag failure to remain
reproducible, accepted XY error below 0.15 m, and any-camera sampled availability above 94% in
each rotation direction. Per-observation metrics are exported to
`build/vision-strategy-revisit/solver-frames.csv`; aggregate results remain in `solver-comparison.csv`.
`staticAccepted` includes the new range gate; `qualityAccepted` additionally includes heading.

Real coprocessor solves with injected gyro biases from +/-2 to +/-6 degrees exercise both sides
of the heading threshold. Wrapped-angle unit cases cover +/-180 and the inclusive 5-degree bound.
Native integration checks startup qualification without heading alignment, alignment from actual
field resets, rejected frames not contributing to startup, and recovery from translation offsets.

The known-heading condition is distinct from completing five-pose solver qualification. Startup
may acquire field heading from five accepted same-camera coprocessor frames, while enabled
qualification alone must never activate a gate against an uninitialized gyro field reference.
The thresholds are experimental and require real camera/timing validation; the simulator is not
evidence of hardware accuracy or immunity to gyro-field-heading faults.

Validation: the full suite passed 132 tests with no failures, errors, or skipped tests. Spotless
passed for the nine Java files changed by this follow-up. At the reported position, the default
strategy's worst accepted XY error was 0.0733 m across both rotation directions, with a usable
camera observation at 344/360 and 345/360 sampled instants. Native CTRE tests recovered from
0.25, 0.5, 1, and 2 m translation offsets after vision loss; heading-inconsistent frames were
rejected, and subsequent clean observations recovered the estimate. These use independent
simulated truth and do not establish hardware performance.

### Subsequent strategy-cutoff investigation

The second reported position, (4.72, 0.59) m at +/-0.74 rad/s, exposes a limitation of the
retained 0.5 rad/s constrained cutoff. Both the Hybrid selector and the constrained solver's
eligibility check exclude constrained PnP above that rate. The existing explicit CONSTRAINED
comparison runs its LOWEST_AMBIGUITY fallback at 0.74 rad/s; it does not compare an actual
constrained solve there. The boundary tests verify the configured behavior, not its optimality.

An isolated copy of production IO changed only the constrained cutoff to 1.0 rad/s. It consumed
the same generated frames as unchanged production, retaining the quality gates and all other
solver rules. With accurate capture heading, combined +/-0.74 results were:

| Position | Current maximum accepted XY error | Candidate maximum | Current / candidate sample coverage |
| --- | ---: | ---: | ---: |
| (4.72, 0.59) | 0.4053 m | 0.0298 m | 700 / 720 |
| (6.996, 2.130) | 0.0732 m | 0.0377 m | 689 / 702 |
| Right trench (4.407, 0.650) | 0.3865 m | 0.0329 m | 720 / 720 |
| Left trench (4.407, 7.279) | 0.1196 m | 0.0752 m | 718 / 718 |

Coverage counts are out of 720 sampled instants per position. These additional trench probes
use the current four-camera 1280x800/70-degree fixture, not the original three-camera fixture.
The passing diagnostic test generated 11,520 frames and compared seven heading conditions;
actual constrained execution was verified (22,176 candidate solves, zero baseline solves).

The candidate is not uniformly better with heading faults. At (6.996, 2.130), a -2-degree
heading bias increased accepted RMSE from 0.0254 to 0.1251 m and maximum error from 0.0946
to 0.2613 m. At the left trench, -2 degrees produced 138 accepted errors over 0.3 m.
Injected +/-10 and +/-20 ms heading-time offsets also expose sensitivity; at 0.74 rad/s,
20 ms corresponds to approximately 0.848 degrees of heading error. Actual field alignment
and correct capture-time heading remain prerequisites for using these solvers reliably.

This was a strategy experiment, not a production cutoff change or full-suite validation of
a new policy. Results and source-equivalence checks are in
`build/constrained-switch-probe-20260927/`. A production revision should preserve startup's
independent five-pose qualification, gate heading-dependent solving on established field
alignment, and validate the expanded rate range against the full existing regression suite.
The solver chain still takes the first successful solve; later subsystem quality rejection
does not retry another strategy on the same frame.

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
| At least two visible tags, similarly facing, absolute rotation ≤90 degrees/s | K → C → T → L |
| At least two visible tags, differently facing, absolute rotation ≤90 degrees/s | C → K → T → L |
| One tag, absolute rotation ≤90 degrees/s, translation ≤0.5 m/s | T → K → C → L |
| One tag, absolute rotation ≤90 degrees/s, translation >0.5 m/s | T → C → K → L |
| At least two tags, absolute rotation >90 degrees/s | C → T → L |
| One tag, absolute rotation >90 degrees/s | T → C → L |

K additionally requires aligned capture-time heading and camera calibration; it is skipped above
pi/2 rad/s (90 degrees/s). Its seed order is C, then L, then aligned capture-time odometry.
T requires aligned capture-time heading and is skipped above 1.0 rad/s (about 57.3 degrees/s),
including when it appears in an attempted chain above that rate. Both reject nonfinite angular
rates. These operating limits apply to both signs and are not hardware-calibrated optima.
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
| `VisionConstrainedMotionReplayTest` | Add current four-camera, two-noise-seed sweep through +/-90 degrees/s, above-limit fallback, translated threshold crossings, and biased/delayed heading comparisons against independent truth. Export actual solver and acceptance metrics. |
| `VisionStrategyLatencyTest` | Add delayed-frame threshold crossings in both directions using capture heading and processing-time angular rate. Verify actual Hybrid/explicit solver choice, timestamps, and constrained accuracy. |
| `VisionDynamicStrategyBakeoffTest` | Remove; replacement eliminates duplicated runs, snapshot reuse, primary-name mislabeling, and weak count-only assertions. |
| `VisionStrategyComparisonSupport` | Remove; no remaining caller, and inferred hybrid recommendations were unsupported. |
| `VisionIOPhotonVisionMetadataTest` | Extend with real default single-tag behavior, missing heading/calibration fallback, ±trig boundary and unsafe rates, startup/restart override, empty-batch clearing. Use production batch processing instead of reflection. Retain actual contributing-tag and failed-frame checks. |
| `VisionPoseStaticScenariosTest` | Retain all eight native CTRE scenarios; count fresh latest snapshots only, add fused-position bound, fail initialization errors, preserve independent truth and stationary controls. This is not an all-camera coverage metric. |
| `VisionPoseStaticTest` | Remove redundant self-referential auto test that skipped when its vision never appeared. Its field pose remains in the eight-scenario suite. |
| `RightSideSafeOdometryTest` | Replace with `RightSideSafeSimulationSmokeTest`: assert auto completion, finite poses, and vision activity. Preserve jump CSV/diagnostics, remove the unsupported claim that several multi-meter jumps represent expected path-boundary resets. Runtime/initialization errors now fail instead of skipping. |
| `VisionFilterStabilityTest` | Retain deterministic noise/adversarial/uncertainty/copy and distinct cross-camera tests. Remove two arithmetic startup duplicates covered by the actual startup integration. Correct the claim that its WPILib estimator is the native CTRE estimator. |
| `VisionDistanceConfidencePolicyTest` | Retain average-versus-max calculation; replace enum-shape assertion with missing-sample rejection and exact 7 m boundary coverage. |
| `VisionStartupReseedTest` | Retain accepted-only, same-camera five-pose qualification, exact fifth snapshot, instability, expiration/recovery, duplicates, all-camera forwarding and diagnostic checks. Add both signs of pitch/roll rejection and five-pose recovery after leveling. Verify actual heading-provider alignment/history eligibility and native angle wrapping across absolute resets. |
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
