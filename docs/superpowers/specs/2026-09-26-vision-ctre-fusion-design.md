# Per-camera CTRE fusion and diagnostic logs

The implementation approved in this conversation replaces the winner-only consensus selector
with independently gated, timestamped camera measurements passed to the existing CTRE estimator.
The same run must produce logs that explain each decision and can be exported for offline analysis.

## Measurement path

1. Read unread camera frames. During startup, prefer coprocessor MultiTag with lowest-ambiguity
   fallback. After five-pose qualification, use the guarded HYBRID solver order, with coprocessor
   seeds for constrained PnP. The subsequent same-frame trench/motion test revision is documented
   in [the strategy audit](../../vision-strategy-test-audit.md). Explicit comparison overrides remain.
2. Preserve frame timestamp, sequence, solver and actual contributing tag IDs. Reject a solved
   coprocessor pose if contributing IDs cannot be validated; never remove a tag from an existing solve.
3. Process observations in capture-time order, breaking exact ties by configured camera order.
   Reject nonfinite, stale, duplicate, out-of-order, out-of-field, excessive-Z, ambiguous single-tag,
   excessive-distance, tilted-robot or unavailable-history measurements. Reject excessive translation
   innovation except during initial disabled localization before the first enable.
4. Supply every accepted observation to CTRE, converting FPGA capture time once at the consumer
   boundary. Use equal X/Y standard deviations with distance-squared, tag count and camera factors;
   single-tag fallback has an explicit penalty and heading standard deviation stays very large.
   Aiming and same-face tag heuristics do not improve measurement quality and do not change trust.
5. Keep separate stable coprocessor-MultiTag startup counts per camera. The first camera reaching
   five accepted stable observations supplies the startup pose. Counts never pool. Automatic
   reseeding occurs only while initially disabled, from that qualifying camera; later enabled or
   disabled transitions cannot reopen startup reseeding. Operator recovery remains explicit.

## Logs

Each pose decision is one atomic JSON record inside the normal WPILOG, including camera, event
sequence, capture and processing timestamps, source frame sequence, solver, tags, raw pose,
reference pose at capture, innovation, supplied standard deviations, decision/reason, robot mode,
startup count and fused pose around submission. Unsupported/nonfinite numbers become JSON null.
No-target/failed-solve frames have separate atomic frame records. Existing pose-array plots remain.
Connection state and active camera configuration are recorded. Log queue diagnostics remain visible.

Run metadata contains build-time Git identity/dirty state, dependency versions, actual loaded field
layout geometry/hash, camera transforms, uncertainty/gating values and schema version. Native DS
logging is enabled. All estimator reset paths log explicit events and invalidate sampled history.

An offline command extracts these records to JSONL without altering the WPILOG. This is diagnostic
export, not deterministic full-robot replay or camera-image reprocessing. Hardware log loss, frame
timing, calibration and uncertainty coefficients still require on-robot validation.

## Verification

Regression tests cover multiple cameras reaching CTRE, duplicate/stale/history rejection, exact
solver metadata, the same-camera five-pose gate and reset source, complete JSON records, build
metadata and WPILOG export. Run focused tests first, then the full suite. Update obsolete consensus
tests to the new contract; correct the known 7 m test fixture boundary while preserving the 7 m gate.
