# Vision Strategy Bakeoff and Offline Hybrid Derivation — Design

Date: 2026-06-09
Status: Draft approved conversationally; awaiting written spec review

## Context

The current vision stack now supports multiple PhotonVision solve strategies:

- `MULTI_TAG_PNP_ON_COPROCESSOR`
- `CONSTRAINED_SOLVEPNP`
- `PNP_DISTANCE_TRIG_SOLVE`
- fallback `LOWEST_AMBIGUITY`

Recent static-scenario testing showed that solver choice materially changes stability. In particular, constrained SolvePnP was generally more stable than trig solve across the existing static side-facing scenarios, but the current evidence set is too narrow to justify a final runtime selector.

The next phase is to expand the regression suite into a solver bakeoff under dynamic motion stress, rank the three single strategies using explicit hard-gate criteria, and derive an initial hybrid selector offline before any production hybrid logic is introduced.

## Problem

We do not yet know which strategy is best under:

1. in-place high-rate rotation,
2. simultaneous translation plus rotation,
3. differing field-side visibility geometry,
4. varying accepted-observation structures during motion.

Choosing a production strategy without this data risks optimizing for one scenario while regressing another.

## Goals

- Compare `MULTI_TAG_PNP_ON_COPROCESSOR`, `CONSTRAINED_SOLVEPNP`, and `PNP_DISTANCE_TRIG_SOLVE` under the same dynamic test scenarios.
- Use a deterministic ranking policy:
  - hard gates first,
  - then optimize within surviving strategies.
- Emit enough structured data to support offline derivation of a first hybrid selector.
- Keep phase 1 comparison-only for production behavior: no runtime hybrid selection yet.
- Make phase 2 straightforward by reusing the same scenarios and metrics to compare the eventual hybrid against the three single strategies.

## Non-goals

- Rewriting the current production vision weighting model.
- Reintroducing mandatory coplanar penalties or broad rejection heuristics.
- Changing camera transforms or field layout configuration.
- Finalizing a production hybrid strategy in this phase without supporting test evidence.

## Decision Policy

Strategy evaluation uses the following order.

### Hard gates

A strategy is disqualified for a scenario set if it violates any of:

- excessive max single-cycle fused odometry jump,
- catastrophic outlier count above threshold,
- loss of accepted-vision coverage below threshold required for meaningful estimation.

These thresholds remain scenario-specific constants in the regression harness and are designed to reject teleporting or near-teleporting behavior before considering average accuracy metrics.

### Ranking among survivors

If multiple strategies pass hard gates, rank them by:

1. lower mean vision deviation from ground truth,
2. lower max vision deviation,
3. better accepted-vision coverage,
4. consistency across scenarios, rather than isolated wins.

## Test Matrix

The bakeoff extends the existing sim-based vision regression harness with strategy-parameterized motion scenarios.

Each scenario is executed once per strategy:

- `MULTI_TAG_PNP_ON_COPROCESSOR`
- `CONSTRAINED_SOLVEPNP`
- `PNP_DISTANCE_TRIG_SOLVE`

All three use `LOWEST_AMBIGUITY` only as a terminal fallback where needed to avoid empty-estimate artifacts unrelated to the primary solver under test.

## New Dynamic Scenarios

### 1. Left in-place rotation

- name: `left_spin`
- start pose: `(x=2.5, y=5.5, yaw=0 deg)`
- motion: in-place rotation
- angular rate: `0.75 rotations/sec`
- translation velocity: zero

Purpose:
Stress ambiguity handling and heading-conditioned solving while the robot remains in one visibility region.

### 2. Right in-place rotation

- name: `right_spin`
- start pose: `(x=2.5, y=2.5, yaw=0 deg)`
- motion: in-place rotation
- angular rate: `0.75 rotations/sec`
- translation velocity: zero

Purpose:
Mirror the left-side case to expose strategy sensitivity to field-side geometry and tag visibility differences.

### 3. Shuttle while spinning

- name: `shuttle_spin`
- initial pose: `(x=2.5, y=5.5, yaw=0 deg)`
- motion: repeated translation between `(2.5, 5.5)` and `(2.5, 2.5)`
- translation speed: `0.8 * TunerConstants.kSpeedAt12Volts`
- angular rate: `0.75 rotations/sec`

Purpose:
Stress simultaneous translation, high angular rate, field-side transitions, and dynamic changes in observed tag sets.

## Metrics Per Scenario x Strategy

Each run records:

- `maxOdomJumpMeters`
- `catastrophicOutlierCount`
- `visionAcceptedCycles`
- `visionCoverageRatio`
- `maxVisionDeviationMeters`
- `meanVisionDeviationMeters`
- optional supporting metrics:
  - cycles executed,
  - max instantaneous innovation rejected or accepted,
  - strategy label in output filenames and summary logs.

CSV logs remain the primary raw artifact. Structured one-line summaries are emitted to stdout for quick comparison and scripted aggregation.

## Harness Design

The current static-scenario regression pattern will be extended rather than replaced.

Key design choices:

- Reuse the same HAL/sim setup pattern already used by the vision tests.
- Add a strategy override mechanism per test class or per test invocation so the same scenario code can run under each strategy without editing production constants between runs.
- Keep motion generation inside the test harness rather than relying on PathPlanner autos for these scenarios. The motion must be deterministic, simple, and directly parameterized by test code.
- Use known ground-truth poses computed from the commanded motion profile so deviation metrics remain explicit and reproducible.

## Motion Model

The scenario runner owns a deterministic simulated motion profile.

For rotation-only cases:

- heading evolves at constant angular velocity of `0.75 rps`.
- translation remains fixed.

For shuttle motion:

- translation oscillates between the two endpoints along the Y-axis at `0.8 * kSpeedAt12Volts`.
- heading evolves continuously at `0.75 rps` for the full run.
- direction reversals at endpoints are deterministic and logged.

The runner measures fused odometry against the known commanded ground-truth pose each cycle.

## Output Artifacts

Per run, write:

- detailed CSV file under `build/vision-stability/`
- summary stdout line for each scenario/strategy
- aggregate comparison summary for the full bakeoff

Filename format should include:

- scenario name,
- strategy name,
- optional test family prefix.

Example pattern:

- `dynamic-left-spin-constrained.csv`
- `dynamic-right-spin-trig.csv`
- `dynamic-shuttle-spin-multitag.csv`

## Offline Hybrid Derivation

Phase 1 ends with a data-backed offline analysis, not production hybrid logic.

The analysis step will inspect which single strategy wins under which conditions, looking for stable rules such as:

- high angular rate favoring constrained or trig solving,
- static or well-conditioned multi-tag visibility favoring multitag,
- field-side or motion-state effects indicating geometry-dependent solver performance,
- low-coverage or unstable-observation cases where one strategy consistently fails hard gates.

The output of this phase is a proposed selector rule set, for example:

- `if angularRate > threshold and vision geometry is weak -> constrained`
- `if strong multi-tag visibility and low innovation -> multitag`
- `if single/weak target conditions dominate and constrained is unavailable -> trig`

This rule set remains offline until validated in phase 2.

## Phase 2 Target

Once phase 1 identifies a reasonable selector, phase 2 will:

- implement a runtime hybrid strategy selector in `VisionIOPhotonVision`,
- run the same static and dynamic scenario suites against:
  - `MULTI_TAG_PNP_ON_COPROCESSOR`,
  - `CONSTRAINED_SOLVEPNP`,
  - `PNP_DISTANCE_TRIG_SOLVE`,
  - `HYBRID`,
- compare the hybrid against all three single strategies using the same hard-gate-first decision policy.

If no selector rule set is clearly justified by phase 1, phase 2 should stop short of production hybridization.

## Risks

- Sim behavior is still coupled to the drivetrain pose estimator, so absolute realism is limited; however, comparative solver behavior under the same harness is still valuable.
- Dynamic scenario thresholds that are too strict may reject all strategies; thresholds must distinguish instability from expected motion-induced estimator behavior.
- Full-speed shuttle motion may stress parts of the drivetrain sim beyond vision alone; logs should separate vision instability from expected path/kinematic transitions.

## Validation Plan

Validation for phase 1 implementation:

1. Compile main and test code.
2. Run the expanded bakeoff with each of the three single strategies.
3. Confirm the scenario summaries and CSV outputs are generated.
4. Confirm the aggregate comparison ranks strategies using the hard-gate-first policy.
5. Review the resulting evidence before implementing any hybrid runtime behavior.

## Expected Deliverables

Phase 1 deliverables:

- expanded dynamic strategy comparison tests,
- structured scenario/strategy summaries,
- aggregate solver comparison output,
- offline-derived first hybrid rule proposal.

Phase 2 deliverables depend on the evidence from phase 1.
