package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import org.junit.jupiter.api.Test;

/** Lightweight microbenchmark for same-loop vision consensus overhead. */
class VisionConsensusPerformanceTest {

  private static final int CAMERA_COUNT = 4;
  private static final int OBSERVATIONS_PER_CAMERA = 2;
  private static final int CANDIDATES_PER_FRAME = CAMERA_COUNT * OBSERVATIONS_PER_CAMERA;
  private static final int FRAME_COUNT = 4096;
  private static final int WARMUP_ITERATIONS = 6000;
  private static final int MEASURE_ITERATIONS = 60_000;

  private final List<VisionSubsystem.ConsensusCandidate> candidateBuffer =
      new ArrayList<>(CANDIDATES_PER_FRAME);

  private double blackhole = 0.0;

  @Test
  void compareCurrentPassThroughWithProductionConsensusRuntime() throws IOException {
    PoseObservation[][] frames = createSyntheticFrames();

    BenchmarkResult current =
        benchmark("current_pass_through", frames, this::runCurrentPassThroughFrame);
    BenchmarkResult consensus =
        benchmark("production_consensus", frames, this::runProductionConsensusFrame);

    double overheadRatio = consensus.nanosecondsPerFrame() / current.nanosecondsPerFrame();
    writeCsv(current, consensus, overheadRatio);

    System.out.printf(
        "[VisionConsensusPerf] current=%.1f ns/frame consensus=%.1f ns/frame overhead=%.2fx"
            + " currentAccepted=%.2f consensusAccepted=%.2f%n",
        current.nanosecondsPerFrame(),
        consensus.nanosecondsPerFrame(),
        overheadRatio,
        current.acceptedPerFrame(),
        consensus.acceptedPerFrame());

    assertEquals(
        current.framesMeasured(),
        consensus.framesMeasured(),
        "Both benchmark paths should process the same frame count");
    assertTrue(blackhole > 0.0, "Benchmark work should not be optimized away");
    assertTrue(consensus.acceptedPerFrame() > 0.0, "Consensus path should select observations");
  }

  private BenchmarkResult benchmark(
      String name, PoseObservation[][] frames, FrameRunner frameRunner) {
    for (int i = 0; i < WARMUP_ITERATIONS; i++) {
      frameRunner.run(frames[i % frames.length]);
    }

    long start = System.nanoTime();
    int accepted = 0;
    for (int i = 0; i < MEASURE_ITERATIONS; i++) {
      accepted += frameRunner.run(frames[i % frames.length]);
    }
    long elapsedNanos = System.nanoTime() - start;

    return new BenchmarkResult(name, MEASURE_ITERATIONS, accepted, elapsedNanos);
  }

  private int runCurrentPassThroughFrame(PoseObservation[] observations) {
    int accepted = 0;
    for (int i = 0; i < observations.length; i++) {
      PoseObservation observation = observations[i];
      if (VisionSubsystem.rejectionReason(observation).isPresent()) {
        continue;
      }

      Matrix<N3, N1> stdDevs =
          VisionSubsystem.standardDeviations(observation, i % CAMERA_COUNT, false);
      consume(observation.pose().toPose2d(), stdDevs);
      accepted++;
    }
    return accepted;
  }

  private int runProductionConsensusFrame(PoseObservation[] observations) {
    candidateBuffer.clear();
    for (int i = 0; i < observations.length; i++) {
      PoseObservation observation = observations[i];
      if (VisionSubsystem.rejectionReason(observation).isPresent()) {
        continue;
      }

      Matrix<N3, N1> stdDevs =
          VisionSubsystem.standardDeviations(observation, i % CAMERA_COUNT, false);
      candidateBuffer.add(
          new VisionSubsystem.ConsensusCandidate(
              i % CAMERA_COUNT,
              "camera" + (i % CAMERA_COUNT),
              "Vision/camera" + (i % CAMERA_COUNT),
              observation,
              observation.pose().toPose2d(),
              stdDevs,
              0.0));
    }

    Optional<VisionSubsystem.ConsensusCandidate> selected =
        VisionSubsystem.selectConsensusCandidate(candidateBuffer);
    selected.ifPresent(candidate -> consume(candidate.visionPose(), candidate.standardDeviations()));
    return selected.isPresent() ? 1 : 0;
  }

  private PoseObservation[][] createSyntheticFrames() {
    Random random = new Random(20260626);
    PoseObservation[][] frames = new PoseObservation[FRAME_COUNT][CANDIDATES_PER_FRAME];
    for (int frame = 0; frame < FRAME_COUNT; frame++) {
      double timestamp = frame * 0.02;
      double baseX = 4.0 + Math.sin(frame * 0.01) * 0.15;
      double baseY = 4.0 + Math.cos(frame * 0.012) * 0.15;

      for (int camera = 0; camera < CAMERA_COUNT; camera++) {
        int offset = camera * OBSERVATIONS_PER_CAMERA;
        frames[frame][offset] =
            observation(
                timestamp,
                baseX + noise(random, 0.04),
                baseY + noise(random, 0.04),
                2,
                2.0,
                new int[] {1, 2});

        boolean outlier = frame % 11 == camera;
        frames[frame][offset + 1] =
            observation(
                timestamp,
                baseX + (outlier ? 1.25 : noise(random, 0.10)),
                baseY + (outlier ? -1.10 : noise(random, 0.10)),
                outlier ? 1 : 2,
                outlier ? 4.0 : 2.4,
                outlier ? new int[] {7} : new int[] {1, 2});
      }
    }
    return frames;
  }

  private PoseObservation observation(
      double timestamp, double x, double y, int tagCount, double distance, int[] tagIds) {
    return new PoseObservation(
        timestamp,
        new Pose3d(x, y, 0.0, new Rotation3d()),
        0.08,
        tagCount,
        distance,
        PoseObservationType.PHOTONVISION,
        tagIds);
  }

  private double noise(Random random, double width) {
    return (random.nextDouble() - 0.5) * width;
  }

  private void consume(Pose2d pose, Matrix<N3, N1> stdDevs) {
    blackhole += pose.getX() * 1e-9 + pose.getY() * 1e-9 + stdDevs.get(0, 0) * 1e-6;
  }

  private void writeCsv(
      BenchmarkResult current, BenchmarkResult consensus, double overheadRatio) throws IOException {
    Path dir = Path.of("build", "vision-stability");
    Files.createDirectories(dir);
    Files.write(
        dir.resolve("vision-consensus-performance.csv"),
        List.of(
            "mode,frames,accepted,total_ns,ns_per_frame,accepted_per_frame",
            current.toCsv(),
            consensus.toCsv(),
            "overhead_ratio,,,,"
                + String.format(java.util.Locale.US, "%.6f", overheadRatio)
                + ","));
  }

  @FunctionalInterface
  private interface FrameRunner {
    int run(PoseObservation[] observations);
  }

  private record BenchmarkResult(
      String name, int framesMeasured, int accepted, long elapsedNanoseconds) {
    double nanosecondsPerFrame() {
      return elapsedNanoseconds / (double) framesMeasured;
    }

    double acceptedPerFrame() {
      return accepted / (double) framesMeasured;
    }

    String toCsv() {
      return String.format(
          java.util.Locale.US,
          "%s,%d,%d,%d,%.3f,%.6f",
          name,
          framesMeasured,
          accepted,
          elapsedNanoseconds,
          nanosecondsPerFrame(),
          acceptedPerFrame());
    }
  }
}
