# Vision Strategy Bakeoff Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add dynamic strategy-comparison vision tests, generate aggregate bakeoff summaries for multitag vs constrained vs trig, and derive an initial offline hybrid recommendation from the test evidence.

**Architecture:** Extend the existing sim-based vision regression harness rather than creating a parallel framework. Add a dynamic scenario runner that can execute the same motion profile under different Photon strategy orders, emit comparable CSV and stdout summaries, and rank strategies with hard-gate-first scoring before any runtime hybrid logic is added.

**Tech Stack:** Java 17, WPILib sim/HAL, PhotonVision, JUnit 5, GradleRIO

---

## File Structure

- Modify: `src/test/java/frc/robot/subsystems/vision/VisionPoseStaticScenariosTest.java`
  - Keep current static scenarios passing and factor reusable summary/CSV helpers only if needed.
- Create: `src/test/java/frc/robot/subsystems/vision/VisionDynamicStrategyBakeoffTest.java`
  - New dynamic scenario harness for left spin, right spin, and shuttle spin.
- Create: `src/test/java/frc/robot/subsystems/vision/VisionStrategyComparisonSupport.java`
  - Shared records/helpers for scenario summaries, hard-gate evaluation, and aggregate ranking.
- Modify: `build.gradle`
  - Add optional task/property wiring only if needed for targeted bakeoff execution or summary runs.
- Modify: `src/main/java/frc/robot/util/constants/VisionConstants.java`
  - Only if a small helper constant is needed for hybrid-analysis thresholds; avoid production hybrid logic in phase 1.

### Task 1: Add comparison support types

**Files:**
- Create: `src/test/java/frc/robot/subsystems/vision/VisionStrategyComparisonSupport.java`
- Test: `src/test/java/frc/robot/subsystems/vision/VisionDynamicStrategyBakeoffTest.java`

- [ ] **Step 1: Write the failing test skeleton**

```java
@Test
void bakeoffSupportCanRankStrategiesHardGatesFirst() {
  var constrained = new VisionStrategyComparisonSupport.StrategySummary(
      "CONSTRAINED_SOLVEPNP", 0.08, 0, 0.92, 0.60, 0.22);
  var trig = new VisionStrategyComparisonSupport.StrategySummary(
      "PNP_DISTANCE_TRIG_SOLVE", 0.20, 1, 0.95, 0.55, 0.18);

  var ranked = VisionStrategyComparisonSupport.rankStrategies(List.of(constrained, trig), 0.15, 0);

  assertEquals("CONSTRAINED_SOLVEPNP", ranked.get(0).strategyName());
  assertTrue(ranked.get(0).passesHardGates());
  assertFalse(ranked.get(1).passesHardGates());
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./gradlew test --tests "frc.robot.subsystems.vision.VisionDynamicStrategyBakeoffTest.bakeoffSupportCanRankStrategiesHardGatesFirst"`
Expected: FAIL because support class/test harness does not exist yet.

- [ ] **Step 3: Write minimal comparison support implementation**

```java
final class VisionStrategyComparisonSupport {
  record StrategySummary(
      String strategyName,
      double maxOdomJumpMeters,
      int catastrophicOutlierCount,
      double visionCoverageRatio,
      double maxVisionDeviationMeters,
      double meanVisionDeviationMeters) {
    boolean passesHardGates(double maxJump, int maxOutliers, double minCoverage) {
      return maxOdomJumpMeters <= maxJump
          && catastrophicOutlierCount <= maxOutliers
          && visionCoverageRatio >= minCoverage;
    }
  }

  record RankedStrategy(StrategySummary summary, boolean passesHardGates) {
    String strategyName() {
      return summary.strategyName();
    }
  }

  static List<RankedStrategy> rankStrategies(
      List<StrategySummary> summaries, double maxJump, int maxOutliers) {
    double minCoverage = 0.50;
    return summaries.stream()
        .map(summary -> new RankedStrategy(summary, summary.passesHardGates(maxJump, maxOutliers, minCoverage)))
        .sorted((left, right) -> {
          if (left.passesHardGates() != right.passesHardGates()) {
            return left.passesHardGates() ? -1 : 1;
          }
          int meanCmp = Double.compare(left.summary.meanVisionDeviationMeters(), right.summary.meanVisionDeviationMeters());
          if (meanCmp != 0) {
            return meanCmp;
          }
          int maxCmp = Double.compare(left.summary.maxVisionDeviationMeters(), right.summary.maxVisionDeviationMeters());
          if (maxCmp != 0) {
            return maxCmp;
          }
          return -Double.compare(left.summary.visionCoverageRatio(), right.summary.visionCoverageRatio());
        })
        .toList();
  }
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./gradlew test --tests "frc.robot.subsystems.vision.VisionDynamicStrategyBakeoffTest.bakeoffSupportCanRankStrategiesHardGatesFirst"`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/test/java/frc/robot/subsystems/vision/VisionStrategyComparisonSupport.java src/test/java/frc/robot/subsystems/vision/VisionDynamicStrategyBakeoffTest.java
git commit -m "test: add vision strategy ranking support"
```

### Task 2: Add failing dynamic bakeoff tests

**Files:**
- Create: `src/test/java/frc/robot/subsystems/vision/VisionDynamicStrategyBakeoffTest.java`
- Modify: `build.gradle`

- [ ] **Step 1: Write failing dynamic scenario tests**

```java
@Test
void leftSpinBakeoffProducesSummariesForAllStrategies() throws IOException {
  var result = runBakeoff(leftSpinScenario());

  assertEquals(3, result.strategySummaries().size());
  assertTrue(result.strategyNames().contains("MULTI_TAG_PNP_ON_COPROCESSOR"));
  assertTrue(result.strategyNames().contains("CONSTRAINED_SOLVEPNP"));
  assertTrue(result.strategyNames().contains("PNP_DISTANCE_TRIG_SOLVE"));
}

@Test
void shuttleSpinBakeoffRanksStrategiesWithHardGatesFirst() throws IOException {
  var result = runBakeoff(shuttleSpinScenario());

  assertFalse(result.rankedStrategies().isEmpty());
  assertTrue(result.rankedStrategies().get(0).passesHardGates());
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `./gradlew test --tests "frc.robot.subsystems.vision.VisionDynamicStrategyBakeoffTest"`
Expected: FAIL because runner/scenarios are not implemented.

- [ ] **Step 3: Add minimal test harness scaffolding**

```java
@Tag("sim")
class VisionDynamicStrategyBakeoffTest {
  private static final double DT = 0.02;
  private static final double SPIN_RATE_RPS = 0.75;
  private static final double SHUTTLE_SPEED_SCALE = 0.8;

  private record DynamicScenario(String name, Pose2d startPose, boolean shuttle) {}

  private static DynamicScenario leftSpinScenario() {
    return new DynamicScenario("left_spin", new Pose2d(2.5, 5.5, Rotation2d.kZero), false);
  }

  private static DynamicScenario shuttleSpinScenario() {
    return new DynamicScenario("shuttle_spin", new Pose2d(2.5, 5.5, Rotation2d.kZero), true);
  }
}
```

- [ ] **Step 4: Run tests to verify scaffolding still fails on missing runner behavior**

Run: `./gradlew test --tests "frc.robot.subsystems.vision.VisionDynamicStrategyBakeoffTest"`
Expected: FAIL on unimplemented `runBakeoff` or missing summaries.

- [ ] **Step 5: Commit**

```bash
git add src/test/java/frc/robot/subsystems/vision/VisionDynamicStrategyBakeoffTest.java build.gradle
git commit -m "test: scaffold dynamic vision bakeoff"
```

### Task 3: Implement strategy-parameterized dynamic runner

**Files:**
- Modify: `src/test/java/frc/robot/subsystems/vision/VisionDynamicStrategyBakeoffTest.java`
- Modify: `build.gradle`

- [ ] **Step 1: Implement strategy override helper and HAL lifecycle**

```java
private static final List<String> STRATEGIES = List.of(
    "MULTI_TAG_PNP_ON_COPROCESSOR,LOWEST_AMBIGUITY",
    "CONSTRAINED_SOLVEPNP,LOWEST_AMBIGUITY",
    "PNP_DISTANCE_TRIG_SOLVE,LOWEST_AMBIGUITY");

private static <T> T withStrategyOrder(String order, ThrowingSupplier<T> supplier) throws Exception {
  String previous = System.getProperty("vision.photon.strategyOrder");
  System.setProperty("vision.photon.strategyOrder", order);
  try {
    return supplier.get();
  } finally {
    if (previous == null) {
      System.clearProperty("vision.photon.strategyOrder");
    } else {
      System.setProperty("vision.photon.strategyOrder", previous);
    }
  }
}
```

- [ ] **Step 2: Implement per-cycle ground-truth motion update and summary capture**

```java
private CycleState advanceGroundTruth(DynamicScenario scenario, int cycle, Pose2d currentTruth) {
  double angularDelta = Units.rotationsToRadians(SPIN_RATE_RPS) * DT;
  Rotation2d nextHeading = currentTruth.getRotation().plus(new Rotation2d(angularDelta));
  if (!scenario.shuttle()) {
    return new CycleState(new Pose2d(currentTruth.getTranslation(), nextHeading), 0.0, false);
  }

  double shuttleSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * SHUTTLE_SPEED_SCALE;
  double deltaY = shuttleSpeed * DT;
  double targetY = movingTowardLow ? 2.5 : 5.5;
  double nextY = MathUtil.clamp(currentTruth.getY() + (movingTowardLow ? -deltaY : deltaY), 2.5, 5.5);
  boolean reversed = Math.abs(nextY - targetY) < 1e-6;
  return new CycleState(new Pose2d(currentTruth.getX(), nextY, nextHeading), shuttleSpeed, reversed);
}
```

- [ ] **Step 3: Implement `runBakeoff` to execute all three strategies and write CSVs**

```java
private BakeoffResult runBakeoff(DynamicScenario scenario) throws Exception {
  List<VisionStrategyComparisonSupport.StrategySummary> summaries = new ArrayList<>();
  for (String order : STRATEGIES) {
    summaries.add(runScenarioForStrategy(scenario, order));
  }
  return new BakeoffResult(
      scenario.name(),
      summaries,
      VisionStrategyComparisonSupport.rankStrategies(summaries, 0.20, 0));
}
```

- [ ] **Step 4: Run the new test file to verify it passes**

Run: `./gradlew test --tests "frc.robot.subsystems.vision.VisionDynamicStrategyBakeoffTest"`
Expected: PASS and CSVs written under `build/vision-stability/`.

- [ ] **Step 5: Commit**

```bash
git add src/test/java/frc/robot/subsystems/vision/VisionDynamicStrategyBakeoffTest.java build.gradle
git commit -m "test: add dynamic vision strategy bakeoff runner"
```

### Task 4: Add aggregate offline hybrid recommendation test/reporting

**Files:**
- Modify: `src/test/java/frc/robot/subsystems/vision/VisionDynamicStrategyBakeoffTest.java`
- Modify: `src/test/java/frc/robot/subsystems/vision/VisionStrategyComparisonSupport.java`

- [ ] **Step 1: Write failing aggregate recommendation test**

```java
@Test
void fullBakeoffProducesOfflineHybridRecommendation() throws Exception {
  var fullResult = runFullBakeoff();

  assertFalse(fullResult.offlineHybridRules().isEmpty());
  assertTrue(fullResult.offlineHybridRules().stream().anyMatch(rule -> rule.contains("angularRate")));
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./gradlew test --tests "frc.robot.subsystems.vision.VisionDynamicStrategyBakeoffTest.fullBakeoffProducesOfflineHybridRecommendation"`
Expected: FAIL because aggregate recommendation does not exist yet.

- [ ] **Step 3: Implement minimal aggregate summary and offline rules**

```java
static List<String> deriveOfflineHybridRules(List<ScenarioResult> results) {
  List<String> rules = new ArrayList<>();
  boolean constrainedWinsSpin = results.stream()
      .filter(result -> result.scenarioName().contains("spin"))
      .allMatch(result -> result.bestStrategy().equals("CONSTRAINED_SOLVEPNP"));
  if (constrainedWinsSpin) {
    rules.add("if angularRateRadPerSec is high, prefer CONSTRAINED_SOLVEPNP");
  }
  boolean multitagWinsLowDeviation = results.stream()
      .anyMatch(result -> result.bestStrategy().equals("MULTI_TAG_PNP_ON_COPROCESSOR"));
  if (multitagWinsLowDeviation) {
    rules.add("if multi-tag visibility is strong and hard gates pass, prefer MULTI_TAG_PNP_ON_COPROCESSOR");
  }
  rules.add("fallback to PNP_DISTANCE_TRIG_SOLVE when heading-conditioned strategies are unavailable");
  return rules;
}
```

- [ ] **Step 4: Run the full bakeoff test class**

Run: `./gradlew visionStabilityTest --tests "frc.robot.subsystems.vision.VisionDynamicStrategyBakeoffTest"`
Expected: PASS with stdout aggregate comparison and offline hybrid recommendation.

- [ ] **Step 5: Commit**

```bash
git add src/test/java/frc/robot/subsystems/vision/VisionDynamicStrategyBakeoffTest.java src/test/java/frc/robot/subsystems/vision/VisionStrategyComparisonSupport.java
git commit -m "test: add offline hybrid recommendation from vision bakeoff"
```

### Task 5: Regression validation across current and new suites

**Files:**
- Modify: `build.gradle` only if task wiring is required after observing validation pain points.
- Test: existing vision regression suite plus new bakeoff tests.

- [ ] **Step 1: Run focused compile validation**

Run: `./gradlew compileJava compileTestJava`
Expected: BUILD SUCCESSFUL.

- [ ] **Step 2: Run the existing static suite and new bakeoff suite together**

Run: `./gradlew visionStabilityTest`
Expected: BUILD SUCCESSFUL with:
- existing static scenario tests still passing,
- `VisionPoseStaticTest` allowed to skip,
- new dynamic bakeoff tests passing.

- [ ] **Step 3: Inspect emitted summaries and confirm hard-gate-first ordering is visible**

Run: `Get-ChildItem build/vision-stability | Where-Object { $_.Name -match 'dynamic|static' } | Select-Object Name`
Expected: dynamic CSVs per scenario/strategy plus existing static artifacts.

- [ ] **Step 4: Update plan/spec references only if implementation diverged materially**

```markdown
If implementation changed thresholds, filenames, or scenario naming, update:
- docs/superpowers/specs/2026-06-09-vision-strategy-bakeoff-design.md
- this plan file only if required for historical accuracy
```

- [ ] **Step 5: Commit**

```bash
git add build.gradle src/test/java/frc/robot/subsystems/vision docs/superpowers/specs/2026-06-09-vision-strategy-bakeoff-design.md
git commit -m "test: complete dynamic vision strategy bakeoff"
```
