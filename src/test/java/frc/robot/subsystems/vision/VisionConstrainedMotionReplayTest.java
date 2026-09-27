package frc.robot.subsystems.vision;

import static frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT;
import static frc.robot.util.constants.VisionConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.*;
import java.util.*;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.*;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Current-camera motion regression, separate from the original three-camera trench fixtures.
 *
 * <p>Images and errors use independent analytic truth, never the estimator's output. The drivetrain
 * supplies capture heading but no translation seed. Heading faults affect only the supplied
 * heading, not the image or frame timestamp. This tests measurement solving and quality gates, not
 * CTRE fusion, startup qualification, image blur, or hardware calibration.
 */
class VisionConstrainedMotionReplayTest {
  private static final Path OUT = Path.of("build/vision-constrained-motion");
  private static final int FRAMES = 120;
  private static final long[] SEEDS = {2375, 9059};
  // Independent contract: do not derive the expected operating boundary from production's constant.
  private static final double NINETY_DEGREES_PER_SECOND = Math.PI / 2;
  private static final String CONSTRAINED = "CONSTRAINED_SOLVEPNP";
  private static final String MULTI = "MULTI_TAG_PNP_ON_COPROCESSOR";
  private static final String[] CAMERAS = {"F", "R", "B", "L"};
  private static final Transform3d[] TRANSFORMS = {
    APTAG_POSE_EST_CAM_F_POS,
    APTAG_POSE_EST_CAM_R_POS,
    APTAG_POSE_EST_CAM_B_POS,
    APTAG_POSE_EST_CAM_L_POS
  };

  private record Position(String name, double x, double y) {}

  private record HeadingCondition(String name, double biasDegrees, double offsetSeconds) {}

  private static final List<HeadingCondition> CONDITIONS =
      List.of(
          new HeadingCondition("exact", 0, 0),
          new HeadingCondition("bias_minus2deg", -2, 0),
          new HeadingCondition("bias_plus2deg", 2, 0),
          new HeadingCondition("time_minus20ms", 0, -.020),
          new HeadingCondition("time_minus10ms", 0, -.010),
          new HeadingCondition("time_plus10ms", 0, .010),
          new HeadingCondition("time_plus20ms", 0, .020));

  private record Motion(Position position, double signedRate, boolean dynamic) {
    String name() {
      return position.name() + (dynamic ? "_translate_ramp_" : "_spin_") + signedRate;
    }

    double duration() {
      return 2 * Math.PI / (dynamic ? 1.35 : Math.abs(signedRate));
    }

    double phase(double t) {
      return 2 * Math.PI * t / duration();
    }

    double yaw(double t) {
      if (!dynamic) return signedRate * t;
      double frequency = 2 * Math.PI / duration();
      return Math.signum(signedRate) * (1.35 * t - .65 * Math.sin(phase(t)) / frequency);
    }

    double omega(double t) {
      return dynamic ? Math.signum(signedRate) * (1.35 - .65 * Math.cos(phase(t))) : signedRate;
    }

    double speed(double t) {
      if (!dynamic) return 0;
      double frequency = 2 * Math.PI / duration();
      return Math.hypot(
          .32 * frequency * Math.cos(phase(t)), .24 * frequency * Math.cos(2 * phase(t)));
    }

    Pose3d truth(double t) {
      return new Pose3d(
          position.x() + (dynamic ? .32 * Math.sin(phase(t)) : 0),
          position.y() + (dynamic ? .12 * Math.sin(2 * phase(t)) : 0),
          0,
          new Rotation3d(0, 0, yaw(t)));
    }

    List<HeadingCondition> conditions() {
      return dynamic
              || Math.abs(signedRate) == .74
              || Math.abs(signedRate) == NINETY_DEGREES_PER_SECOND
          ? CONDITIONS
          : CONDITIONS.subList(0, 1);
    }
  }

  private record Result(VisionIO.PoseObservation observation, double error, boolean accepted) {}

  private static final class Metrics {
    int frames, targets, solved, accepted, staticAccepted, acceptedConstrained, hiddenBias;
    int constrainedBeforePeak, constrainedAfterPeak;
    double squared, rawSquared, max, rawMax, constrainedSquared, constrainedMax, eligibleMax;
    final List<Double> errors = new ArrayList<>();
    final Set<Integer> acceptedInstants = new HashSet<>();
    final Map<String, Integer> solvers = new TreeMap<>();
    final Map<String, Integer> rejections = new TreeMap<>();

    void add(
        PhotonPipelineResult frame,
        Result result,
        Pose2d reference,
        int instant,
        double omega,
        int index) {
      frames++;
      if (frame.hasTargets()) targets++;
      if (result.observation() == null) return;
      var obs = result.observation();
      solved++;
      solvers.merge(obs.solver(), 1, Integer::sum);
      rawSquared += result.error() * result.error();
      rawMax = Math.max(rawMax, result.error());
      if (VisionSubsystem.rejectionReason(obs).isEmpty()) staticAccepted++;
      var rejection = VisionSubsystem.rejectionReason(obs, reference, true);
      if (rejection.isPresent()) {
        rejections.merge(rejection.get().split("=")[0], 1, Integer::sum);
        return;
      }
      accepted++;
      squared += result.error() * result.error();
      max = Math.max(max, result.error());
      errors.add(result.error());
      acceptedInstants.add(instant);
      if (Math.abs(omega) <= NINETY_DEGREES_PER_SECOND)
        eligibleMax = Math.max(eligibleMax, result.error());
      if (obs.solver().equals(CONSTRAINED)) {
        acceptedConstrained++;
        constrainedSquared += result.error() * result.error();
        constrainedMax = Math.max(constrainedMax, result.error());
        if (index < FRAMES / 2) constrainedBeforePeak++;
        else constrainedAfterPeak++;
        if (result.error() > .1
            && Math.abs(
                    obs.pose().toPose2d().getRotation().minus(reference.getRotation()).getDegrees())
                < 1) hiddenBias++;
      }
    }

    double rmse() {
      return accepted == 0 ? Double.NaN : Math.sqrt(squared / accepted);
    }

    double constrainedRmse() {
      return acceptedConstrained == 0
          ? Double.NaN
          : Math.sqrt(constrainedSquared / acceptedConstrained);
    }

    double p95() {
      if (errors.isEmpty()) return Double.NaN;
      errors.sort(Double::compare);
      return errors.get((int) Math.ceil(errors.size() * .95) - 1);
    }

    void write(
        BufferedWriter writer,
        Motion motion,
        HeadingCondition condition,
        String policy,
        String camera)
        throws IOException {
      csv(
          writer,
          motion.name(),
          condition.name(),
          policy,
          camera,
          frames,
          targets,
          solved,
          accepted,
          staticAccepted,
          acceptedInstants.size(),
          acceptedInstants.size() / (double) (FRAMES * SEEDS.length),
          rmse(),
          p95(),
          max,
          solved == 0 ? Double.NaN : Math.sqrt(rawSquared / solved),
          rawMax,
          acceptedConstrained,
          constrainedRmse(),
          constrainedMax,
          eligibleMax,
          hiddenBias,
          solvers,
          rejections);
    }
  }

  private static final class Paired {
    int gains, losses, shared, betterBy5cm, worseBy5cm;
    double maxRegression;

    void add(Result selected, Result coprocessor) {
      if (selected.accepted() && !coprocessor.accepted()) gains++;
      if (!selected.accepted() && coprocessor.accepted()) losses++;
      if (selected.accepted() && coprocessor.accepted()) {
        shared++;
        double delta = selected.error() - coprocessor.error();
        if (delta < -.05) betterBy5cm++;
        if (delta > .05) worseBy5cm++;
        maxRegression = Math.max(maxRegression, delta);
      }
    }
  }

  @Test
  void actualConstrainedSolvesRemainAccurateThroughNinetyDegreesPerSecondAndExposeHeadingFaults()
      throws Exception {
    assertTrue(HAL.initialize(500, 0));
    String oldOrder = System.getProperty("vision.photon.strategyOrder");
    String oldMode = System.getProperty("vision.photon.strategyMode");
    Files.createDirectories(OUT);
    var targets =
        APTAG_FIELD_LAYOUT.getTags().stream()
            .map(tag -> new VisionTargetSim(tag.pose, TargetModel.kAprilTag36h11, tag.ID))
            .toList();
    var motions = motions();
    int exercised = 0;
    try (var frames = Files.newBufferedWriter(OUT.resolve("frames.csv"));
        var summaries = Files.newBufferedWriter(OUT.resolve("summary.csv"));
        var pairs = Files.newBufferedWriter(OUT.resolve("paired-tradeoffs.csv"))) {
      frames.write(
          "scenario,condition,policy,camera,seed,frame,captureTimeSeconds,omegaRadPerSec,linearSpeedMps,headingBiasDeg,headingTimeOffsetSec,effectiveHeadingErrorDeg,truthX,truthY,truthYawRad,suppliedYawRad,visibleTagCount,solved,solver,tagCount,tagIds,distanceM,sigmaXY,rawX,rawY,rawZ,rawYawRad,yawMinusSuppliedDeg,errorM,staticAccepted,qualityAccepted,rejection,coprocReprojectionError,frameStatus\n");
      summaries.write(
          "scenario,condition,policy,camera,frames,framesWithTargets,solved,accepted,staticAccepted,instantsWithObservation,instantCoverage,acceptedRmseM,acceptedP95M,acceptedMaxM,rawRmseM,rawMaxM,acceptedConstrained,constrainedRmseM,constrainedMaxM,eligibleRateMaxM,acceptedHiddenBiasOver10cm,actualSolvers,rejections\n");
      pairs.write(
          "scenario,condition,acceptanceGains,acceptanceLosses,sharedAccepted,improvedOver5cm,worsenedOver5cm,maxRegressionM\n");
      System.clearProperty("vision.photon.strategyMode");
      for (var motion : motions) {
        var conditions = motion.conditions();
        Metrics[][] totals = metrics(conditions.size());
        Paired[] paired = new Paired[conditions.size()];
        Arrays.setAll(paired, ignored -> new Paired());
        for (int camera = 0; camera < TRANSFORMS.length; camera++) {
          var io =
              new VisionIOPhotonVision(
                  "motion-replay-" + motion.name() + "-" + CAMERAS[camera], TRANSFORMS[camera]);
          var properties = new SimCameraProperties();
          properties.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
          properties.setCalibError(.38, .1);
          Metrics[][] perCamera = metrics(conditions.size());
          try (var sim = new PhotonCameraSim(io.camera, properties, APTAG_FIELD_LAYOUT)) {
            sim.enableRawStream(false);
            sim.enableProcessedStream(false);
            sim.enableDrawWireframe(false);
            io.markVisionInitializationComplete();
            for (int seedIndex = 0; seedIndex < SEEDS.length; seedIndex++) {
              long seed = SEEDS[seedIndex];
              properties.setRandomSeed(seed + camera);
              for (int index = 0; index < FRAMES; index++) {
                double t = motion.duration() * index / (FRAMES - 1);
                var truth = motion.truth(t);
                var generated = sim.process(10, truth.transformBy(TRANSFORMS[camera]), targets);
                long micros = 10_000_000 + Math.round(t * 1_000_000);
                var frame =
                    new PhotonPipelineResult(
                        index,
                        micros,
                        micros + 10_000,
                        0,
                        generated.getTargets(),
                        generated.getMultiTagResult());
                sim.submitProcessedFrame(frame);
                for (int c = 0; c < conditions.size(); c++) {
                  var condition = conditions.get(c);
                  // Pose history supplies an orientation, not accumulated turns. Rotation2d(double)
                  // preserves unwrapped radians; normalize through the independent analytic pose.
                  var heading =
                      motion
                          .truth(t + condition.offsetSeconds())
                          .getRotation()
                          .toRotation2d()
                          .plus(Rotation2d.fromDegrees(condition.biasDegrees()));
                  // Acceptance intentionally sees the same potentially faulty heading as the solve.
                  var reference = new Pose2d(truth.toPose2d().getTranslation(), heading);
                  io.setHeadingProvider(
                      headingProvider(frame, heading, motion.omega(t), motion.speed(t)));
                  Result[] results = new Result[2];
                  for (int policy = 0; policy < 2; policy++) {
                    if (policy == 0) System.clearProperty("vision.photon.strategyOrder");
                    else
                      System.setProperty(
                          "vision.photon.strategyOrder", MULTI + ",LOWEST_AMBIGUITY");
                    var inputs = new VisionIO.VisionIOInputs();
                    io.processResults(List.of(frame), inputs);
                    var result = result(frame, inputs, truth, reference, motion.omega(t));
                    results[policy] = result;
                    int instant = seedIndex * FRAMES + index;
                    perCamera[c][policy].add(
                        frame, result, reference, instant, motion.omega(t), index);
                    totals[c][policy].add(
                        frame, result, reference, instant, motion.omega(t), index);
                    writeFrame(
                        frames,
                        motion,
                        condition,
                        policyName(policy),
                        camera,
                        seed,
                        t,
                        frame,
                        inputs,
                        result,
                        truth,
                        reference);
                    if (policy == 0
                        && result.accepted()
                        && result.observation().solver().equals(CONSTRAINED)) {
                      double headingError =
                          Math.abs(heading.minus(truth.toPose2d().getRotation()).getRadians());
                      // Global accepted range 7 m + camera lever arm; allow twice first-order
                      // range*angle
                      // for planar geometry plus 10 cm for corner noise. This is a bound, not a
                      // claim
                      // that gyro-biased constrained measurements are as accurate as exact-heading
                      // ones.
                      assertTrue(
                          result.error() < .10 + 2 * 7.5 * headingError,
                          motion.name()
                              + " "
                              + condition.name()
                              + " constrained error="
                              + result.error());
                    }
                  }
                  paired[c].add(results[0], results[1]);
                }
              }
            }
          } finally {
            io.camera.close();
          }
          for (int c = 0; c < conditions.size(); c++)
            for (int policy = 0; policy < 2; policy++)
              perCamera[c][policy].write(
                  summaries, motion, conditions.get(c), policyName(policy), CAMERAS[camera]);
        }
        for (int c = 0; c < conditions.size(); c++) {
          for (int policy = 0; policy < 2; policy++)
            totals[c][policy].write(
                summaries, motion, conditions.get(c), policyName(policy), "ALL");
          var pair = paired[c];
          csv(
              pairs,
              motion.name(),
              conditions.get(c).name(),
              pair.gains,
              pair.losses,
              pair.shared,
              pair.betterBy5cm,
              pair.worseBy5cm,
              pair.maxRegression);
        }
        // Flush complete scenario evidence even when the following regression assertion fails.
        frames.flush();
        summaries.flush();
        pairs.flush();
        verifyMotion(motion, totals);
        exercised += conditions.size();
        System.out.printf(
            Locale.ROOT,
            "CONSTRAINED_MOTION %s exact RMSE=%.6f max=%.6f coverage=%d/%d actualConstrained=%d%n",
            motion.name(),
            totals[0][0].rmse(),
            totals[0][0].max,
            totals[0][0].acceptedInstants.size(),
            FRAMES * SEEDS.length,
            totals[0][0].solvers.getOrDefault(CONSTRAINED, 0));
      }
      assertEquals(80, motions.size());
      assertEquals(224, exercised);
    } finally {
      restore("vision.photon.strategyOrder", oldOrder);
      restore("vision.photon.strategyMode", oldMode);
    }
  }

  private static List<Motion> motions() {
    var motions = new ArrayList<Motion>();
    for (var position :
        List.of(
            new Position("reported_new", 4.72, .59),
            new Position("reported_earlier", 6.996, 2.130),
            new Position("trench_right", 4.407, .650),
            new Position("trench_left", 4.407, 7.279))) {
      for (int direction : new int[] {-1, 1}) {
        // Start at the reported .74 rad/s so restoring the former .5 cutoff fails on real solves.
        for (double rate :
            new double[] {.74, .5, 1, 1.25, 1.5, Math.PI / 2, Math.PI / 2 + .00001, 2, 4.7})
          motions.add(new Motion(position, direction * rate, false));
        motions.add(new Motion(position, direction, true));
      }
    }
    return motions;
  }

  private static Metrics[][] metrics(int conditions) {
    Metrics[][] metrics = new Metrics[conditions][2];
    for (var pair : metrics) Arrays.setAll(pair, ignored -> new Metrics());
    return metrics;
  }

  private static VisionIOPhotonVision.VisionHeadingProvider headingProvider(
      PhotonPipelineResult frame, Rotation2d heading, double omega, double speed) {
    return new VisionIOPhotonVision.VisionHeadingProvider() {
      public Optional<Rotation2d> getHeadingAtTimestamp(double timestamp) {
        assertEquals(frame.getTimestampSeconds(), timestamp, 1e-9);
        return Optional.of(heading);
      }

      public Optional<Pose3d> getSeedPoseAtTimestamp(double timestamp) {
        // No privileged truth translation is available to the production solver.
        assertEquals(frame.getTimestampSeconds(), timestamp, 1e-9);
        return Optional.empty();
      }

      public double getAngularRateRadPerSec() {
        return omega;
      }

      public double getLinearSpeedMetersPerSecond() {
        return speed;
      }
    };
  }

  private static Result result(
      PhotonPipelineResult frame,
      VisionIO.VisionIOInputs inputs,
      Pose3d truth,
      Pose2d reference,
      double omega) {
    var observations = inputs.getPoseObservations();
    assertTrue(
        observations.length <= 1, "One camera frame must contribute at most one observation");
    if (observations.length == 0) return new Result(null, Double.NaN, false);
    var obs = observations[0];
    assertEquals(frame.metadata.sequenceID, obs.frameSequenceId());
    assertEquals(frame.getTimestampSeconds(), obs.timestamp(), 1e-9);
    if (Math.abs(omega) > NINETY_DEGREES_PER_SECOND)
      assertNotEquals(CONSTRAINED, obs.solver(), "Constrained must stop above 90 degrees/sec");
    double error =
        obs.pose().toPose2d().getTranslation().getDistance(truth.toPose2d().getTranslation());
    assertTrue(Double.isFinite(error));
    return new Result(obs, error, VisionSubsystem.rejectionReason(obs, reference, true).isEmpty());
  }

  private static void verifyMotion(Motion motion, Metrics[][] metrics) {
    var exact = metrics[0][0];
    var coprocessor = metrics[0][1];
    String label = motion.name();
    boolean entirelyEligible =
        !motion.dynamic() && Math.abs(motion.signedRate()) <= NINETY_DEGREES_PER_SECOND;
    assertTrue(
        exact.acceptedInstants.size() >= (entirelyEligible ? .94 : .90) * FRAMES * SEEDS.length,
        label + " coverage=" + exact.acceptedInstants.size());
    assertTrue(exact.accepted >= .95 * coprocessor.accepted, label + " lost useful observations");
    assertTrue(exact.rmse() < (entirelyEligible ? .035 : .15), label + " RMSE=" + exact.rmse());
    assertTrue(exact.max < (entirelyEligible ? .15 : .60), label + " max=" + exact.max);
    if (entirelyEligible || motion.dynamic()) {
      assertTrue(
          exact.solvers.getOrDefault(CONSTRAINED, 0) > 20,
          label + " must execute actual constrained PnP, not merely select a named policy");
      assertTrue(
          exact.constrainedRmse() < .025, label + " constrained RMSE=" + exact.constrainedRmse());
      assertTrue(exact.eligibleMax < .15, label + " eligible-rate max=" + exact.eligibleMax);
    } else {
      assertEquals(0, exact.solvers.getOrDefault(CONSTRAINED, 0), label);
      assertEquals(coprocessor.solvers, exact.solvers, label + " fast fallback solver regression");
      assertEquals(coprocessor.rmse(), exact.rmse(), 1e-9, label);
    }
    if (motion.dynamic()) {
      assertTrue(
          exact.constrainedBeforePeak > 0,
          label + " must solve before acceleration crosses the limit");
      assertTrue(
          exact.constrainedAfterPeak > 0,
          label + " must resume constrained solving after deceleration");
    }
    for (int c = 1; c < metrics.length; c++) {
      var fault = metrics[c][0];
      var condition = motion.conditions().get(c);
      assertTrue(
          fault.acceptedConstrained > 20,
          label + " " + condition.name() + " must exercise biased solves");
      assertTrue(
          fault.constrainedRmse() > exact.constrainedRmse() + .01,
          label + " " + condition.name() + " must expose heading-induced position error");
      if (condition.biasDegrees() != 0)
        assertTrue(
            fault.hiddenBias > 0,
            label
                + " biased constrained poses can pass the shared-heading gate with over 10 cm"
                + " error");
    }
  }

  private static void writeFrame(
      BufferedWriter writer,
      Motion motion,
      HeadingCondition condition,
      String policy,
      int camera,
      long seed,
      double t,
      PhotonPipelineResult frame,
      VisionIO.VisionIOInputs inputs,
      Result result,
      Pose3d truth,
      Pose2d reference)
      throws IOException {
    List<Object> row =
        new ArrayList<>(
            List.of(
                motion.name(),
                condition.name(),
                policy,
                CAMERAS[camera],
                seed,
                frame.metadata.sequenceID,
                frame.getTimestampSeconds(),
                motion.omega(t),
                motion.speed(t),
                condition.biasDegrees(),
                condition.offsetSeconds(),
                reference.getRotation().minus(truth.toPose2d().getRotation()).getDegrees(),
                truth.getX(),
                truth.getY(),
                truth.getRotation().getZ(),
                reference.getRotation().getRadians(),
                frame.getTargets().size()));
    var obs = result.observation();
    if (obs == null) {
      row.addAll(
          List.of(
              false,
              "",
              0,
              "",
              Double.NaN,
              Double.NaN,
              Double.NaN,
              Double.NaN,
              Double.NaN,
              Double.NaN,
              Double.NaN,
              Double.NaN,
              false,
              false,
              "NO_POSE"));
    } else {
      row.addAll(
          List.of(
              true,
              obs.solver(),
              obs.tagCount(),
              Arrays.stream(obs.tagIDs())
                  .mapToObj(Integer::toString)
                  .collect(Collectors.joining("|")),
              obs.averageTagDistance(),
              VisionSubsystem.standardDeviations(obs, camera, false).get(0, 0),
              obs.pose().getX(),
              obs.pose().getY(),
              obs.pose().getZ(),
              obs.pose().getRotation().getZ(),
              obs.pose().toPose2d().getRotation().minus(reference.getRotation()).getDegrees(),
              result.error(),
              VisionSubsystem.rejectionReason(obs).isEmpty(),
              result.accepted(),
              VisionSubsystem.rejectionReason(obs, reference, true).orElse("")));
    }
    row.add(frame.getMultiTagResult().map(m -> m.estimatedPose.bestReprojErr).orElse(Double.NaN));
    row.add(
        Arrays.stream(inputs.getFrameDiagnostics())
            .map(VisionIO.FrameDiagnostic::status)
            .collect(Collectors.joining("|")));
    assertEquals(34, row.size());
    csv(writer, row.toArray());
  }

  private static String policyName(int policy) {
    return policy == 0 ? "DEFAULT_HYBRID" : "COPROCESSOR_FALLBACK";
  }

  private static void csv(BufferedWriter writer, Object... values) throws IOException {
    for (int i = 0; i < values.length; i++) {
      if (i != 0) writer.write(',');
      writer.write('"');
      writer.write(values[i].toString().replace("\"", "\"\""));
      writer.write('"');
    }
    writer.newLine();
  }

  private static void restore(String key, String value) {
    if (value == null) System.clearProperty(key);
    else System.setProperty(key, value);
  }
}
