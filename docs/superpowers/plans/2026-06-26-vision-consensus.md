# Vision Consensus Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Filter same-loop multi-camera vision observations through a lightweight consensus selector before feeding CTRE pose estimation.

**Architecture:** `VisionSubsystem` will keep the current per-observation rejection and std-dev math, but collect passing observations into consensus candidates. A selector will choose the largest agreeing XY cluster and feed one representative measurement to the drivetrain, while single-candidate frames still pass through.

**Tech Stack:** Java, WPILib command-based robot code, CTRE drivetrain pose estimator, JUnit 5.

---

### Task 1: Add Consensus Selector Tests

**Files:**
- Modify: `src/test/java/frc/robot/subsystems/vision/VisionFilterStabilityTest.java`

- [ ] **Step 1: Write failing tests**

Add tests proving:
- a single candidate is selected,
- a two-observation cluster beats a far outlier,
- tied cluster size chooses the lower-std-dev candidate.

- [ ] **Step 2: Run the focused test**

Run: `.\gradlew.bat test --tests frc.robot.subsystems.vision.VisionFilterStabilityTest`

Expected before production code: compile failure because consensus candidate/selector types do not exist yet.

### Task 2: Implement Consensus Selection

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/vision/VisionSubsystem.java`
- Modify: `src/main/java/frc/robot/util/constants/VisionConstants.java`

- [ ] **Step 1: Add constants**

Add a consensus radius in meters to `VisionConstants`.

- [ ] **Step 2: Add candidate model and selector**

Add a package-private candidate record/class and selector that:
- returns empty for no candidates,
- returns the only candidate for one candidate,
- scores each candidate by cluster size within the radius,
- breaks ties by lower std-dev/disagreement quality.

- [ ] **Step 3: Wire periodic flow**

Change `processObservation` to return a candidate instead of immediately calling the consumer. After all cameras update, select consensus and feed only that measurement to CTRE. Non-selected candidates are logged as rejected for review.

### Task 3: Verify, Commit, Push, PR

**Files:**
- Commit only consensus-related source, test, benchmark, and plan files.

- [ ] **Step 1: Run focused verification**

Run:
- `.\gradlew.bat test --tests frc.robot.subsystems.vision.VisionFilterStabilityTest`
- `.\gradlew.bat test --tests frc.robot.subsystems.vision.VisionConsensusPerformanceTest`
- `.\gradlew.bat test --tests frc.robot.subsystems.vision.*`
- `git diff --check`

- [ ] **Step 2: Commit and publish**

Use an explicit staged file list, commit as `feat(vision): add consensus filtering`, push `feature/vision-adjustments`, and open a draft PR.
