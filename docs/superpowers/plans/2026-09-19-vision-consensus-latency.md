# Vision consensus and latency experiment

Goal: reduce erroneous in-auto vision corrections without waiting for synchronized camera frames or adding unbounded work on roboRIO 2.

Design approved in conversation: bounded frame processing, one vote per camera, time-aligned agreement, conservative single-camera updates, explicit disagreement rejection, trustworthy solve metadata, and measured simulation results. Preserve manual reset and startup-disabled reseed behavior. Keep the existing MultiTag -> Constrained -> Trig -> Lowest Ambiguity noncoplanar order.

## Implementation and validation

- [x] Capture baseline vision simulation results and artifacts.
- [x] Add regression cases for duplicate-camera voting, split/bridged clusters, timestamp alignment, lone-camera recovery, frame backlog, and true solver target metadata; observe failures before implementation.
- [x] Bound each camera to its newest fresh frame before solving; reject repeated/out-of-order timestamps. Experimental freshness window: 250 ms; future tolerance: 20 ms. Do not wait or retry older frames if the selected solve fails.
- [x] Align XY agreement using capture-time odometry translation differences, retaining original pose/timestamp for fusion. Reject unavailable history. Select a unique largest pairwise-agreeing strict majority of available cameras; allow a lone camera with 2x translation standard deviation and <=0.75 m innovation while enabled. Keep the 2.5 m overall enabled gate. No additional frame-confirmation delay.
- [x] Correct single-target fallback metadata and coprocessor IDs-used filtering; use AprilTag +X face normals.
- [x] Log input/skipped/solved frame counts, solver time, selected age, supporting camera count, and decision reason.
- [x] Run deterministic fault-injection estimator simulation, existing Phoenix/PhotonVision simulations, focused timing benchmarks, and full test suite. Keep failures/skips visible; desktop timing is not a roboRIO timing claim.
- [x] Review patch independently; fix important findings and report residual limits.

Files: VisionIOPhotonVision.java (frame/solve policy); VisionSubsystem.java (filter, alignment, consensus); VisionConstants.java (experimental thresholds); focused and simulation tests; this plan and a results report.

No deploy, merge, or automatic publication of experimental code. Keep edits on experiment/vision-consensus-latency.

Review correction: retain disabled innovation bypass and manual recovery after repositioning. Automatic disabled hard reseeding remains startup-only; new solo correction restrictions apply while enabled.
