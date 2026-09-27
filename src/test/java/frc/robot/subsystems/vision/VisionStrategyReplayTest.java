package frc.robot.subsystems.vision;

import static frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT;
import static frc.robot.util.constants.VisionConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import java.nio.file.*;
import java.util.*;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.*;
import org.photonvision.targeting.PhotonPipelineResult;

/** Solver accuracy against independent capture-time truth, before estimator weighting. */
class VisionStrategyReplayTest {
  private static final int FRAMES = 80;
  private static final long[] SEEDS = {2375, 9059};

  private record Scenario(
      String name,
      double x,
      double y,
      double yaw,
      double omega,
      boolean shuttle,
      double headingBias,
      boolean reportedRotation) {
    Scenario(String name, double y, double yaw, double omega, boolean shuttle) {
      this(name, y, yaw, omega, shuttle, 0);
    }

    Scenario(String name, double y, double yaw, double omega, boolean shuttle, double headingBias) {
      this(name, 4.407, y, yaw, omega, shuttle, headingBias, false);
    }

    int frames() {
      return reportedRotation ? 180 : FRAMES;
    }

    double dt() {
      return reportedRotation ? 2 * Math.PI / Math.abs(omega) / (frames() - 1) : .05;
    }

    Pose3d truth(double t) {
      return new Pose3d(
          shuttle ? 2.5 : x,
          shuttle ? 4 + 1.5 * Math.cos(2 * t) : y,
          0,
          new Rotation3d(0, 0, Math.toRadians(yaw) + omega * t));
    }
  }

  private record Policy(String name, String order) {}

  private static final List<Policy> POLICIES =
      List.of(
          new Policy("MULTI", "MULTI_TAG_PNP_ON_COPROCESSOR,LOWEST_AMBIGUITY"),
          new Policy("CONSTRAINED", "CONSTRAINED_SOLVEPNP,LOWEST_AMBIGUITY"),
          new Policy("TRIG", "PNP_DISTANCE_TRIG_SOLVE,LOWEST_AMBIGUITY"),
          new Policy("HYBRID", ""),
          new Policy("DEFAULT", ""));

  private static class Metrics {
    int frames, targets, solved, staticAccepted, accepted, rejectedAccurate, largeRawCoprocessor;
    double sumSquared, max, rawSumSquared, rawMax;
    final Map<String, Integer> solvers = new TreeMap<>();
    final Map<String, Integer> rejections = new TreeMap<>();
    final List<Double> errors = new ArrayList<>();
    final Set<String> acceptedSamples = new HashSet<>();

    void add(
        PhotonPipelineResult frame,
        VisionIO.VisionIOInputs inputs,
        Pose3d truth,
        Pose2d captureReference,
        String sampleKey) {
      frames++;
      if (frame.hasTargets()) targets++;
      for (var obs : inputs.getPoseObservations()) {
        solved++;
        solvers.merge(obs.solver(), 1, Integer::sum);
        assertEquals(frame.metadata.sequenceID, obs.frameSequenceId());
        assertEquals(frame.getTimestampSeconds(), obs.timestamp(), 1e-9);
        double error =
            obs.pose().toPose2d().getTranslation().getDistance(truth.toPose2d().getTranslation());
        rawSumSquared += error * error;
        rawMax = Math.max(rawMax, error);
        boolean staticPass = VisionSubsystem.rejectionReason(obs).isEmpty();
        if (staticPass) staticAccepted++;
        var rejection = VisionSubsystem.rejectionReason(obs, captureReference, true);
        if (error > 1
            && obs.tagCount() == 2
            && obs.solver().equals("MULTI_TAG_PNP_ON_COPROCESSOR")) {
          largeRawCoprocessor++;
          assertTrue(rejection.isPresent(), "A large two-tag PnP error must be rejected");
        }
        if (rejection.isPresent()) {
          rejections.merge(rejection.get().split("=")[0], 1, Integer::sum);
          if (staticPass && error < .1) rejectedAccurate++;
        } else {
          assertTrue(Double.isFinite(error));
          accepted++;
          sumSquared += error * error;
          max = Math.max(max, error);
          errors.add(error);
          acceptedSamples.add(sampleKey);
        }
      }
    }

    double rmse() {
      return accepted == 0 ? Double.NaN : Math.sqrt(sumSquared / accepted);
    }

    double p95() {
      if (errors.isEmpty()) return Double.NaN;
      errors.sort(Double::compare);
      return errors.get((int) Math.ceil(errors.size() * .95) - 1);
    }

    String csv(String scenario, String policy, String camera) {
      return String.format(
          Locale.ROOT,
          "%s,%s,%s,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,\"%s\",\"%s\",%d,%d,%d",
          scenario,
          policy,
          camera,
          frames,
          targets,
          solved,
          accepted,
          rmse(),
          p95(),
          max,
          solved == 0 ? Double.NaN : Math.sqrt(rawSumSquared / solved),
          rawMax,
          solvers,
          rejections,
          staticAccepted,
          acceptedSamples.size(),
          rejectedAccurate);
    }
  }

  @Test
  void compareIdenticalFramesAtBothTrenchesAndDuringMotion() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    String oldOrder = System.getProperty("vision.photon.strategyOrder");
    String oldMode = System.getProperty("vision.photon.strategyMode");
    List<String> csv =
        new ArrayList<>(
            List.of(
                "scenario,policy,camera,frames,framesWithTargets,solved,qualityAccepted,acceptedRmseM,acceptedP95M,acceptedMaxM,allSolvedRmseM,allSolvedMaxM,actualSolvers,rejections,staticAccepted,instantsWithObservation,rejectedAccurateUnder10cm"));
    List<String> frameCsv =
        new ArrayList<>(
            List.of(
                "scenario,fixture,policy,camera,seed,frame,captureTimeSeconds,omegaRadPerSec,headingBiasDeg,truthX,truthY,truthYawRad,visibleTagCount,solved,solver,tagCount,tagIds,distanceM,sigmaXY,ambiguity,rawX,rawY,rawZ,rawYawRad,rawYawMinusCaptureGyroDegrees,rawErrorM,staticAccepted,qualityAccepted,rejection,coprocBestReprojErr,frameStatus"));
    List<Scenario> scenarios = new ArrayList<>();
    for (String side : List.of("left", "right")) {
      double y = side.equals("left") ? 7.279 : .650;
      for (int yaw : new int[] {-90, 0, 90, 180})
        scenarios.add(new Scenario(side + "_static_" + yaw, y, yaw, 0, false));
      for (double omega : new double[] {.25, .5, .75, Math.PI * 1.5})
        scenarios.add(
            new Scenario(side + "_spin_" + omega, y, side.equals("left") ? -90 : 90, omega, false));
    }
    for (double bias : new double[] {-2, 2}) {
      scenarios.add(new Scenario("left_bias_" + bias, 7.279, -90, .25, false, bias));
      scenarios.add(new Scenario("right_bias_" + bias, .650, 90, .25, false, bias));
    }
    scenarios.add(new Scenario("shuttle_spin", 4, 0, Math.PI * 1.5, true));
    for (double omega : new double[] {-.74, .74})
      scenarios.add(
          new Scenario("reported_rotate_" + omega, 6.996, 2.130, 0, omega, false, 0, true));
    var targets =
        APTAG_FIELD_LAYOUT.getTags().stream()
            .map(tag -> new VisionTargetSim(tag.pose, TargetModel.kAprilTag36h11, tag.ID))
            .toList();
    Transform3d[] transforms = {
      APTAG_POSE_EST_CAM_F_POS,
      APTAG_POSE_EST_CAM_R_POS,
      APTAG_POSE_EST_CAM_B_POS,
      APTAG_POSE_EST_CAM_L_POS
    };
    String[] names = {"F", "R", "B", "L"};
    var reportedRotations = new Metrics();
    try {
      for (var scenario : scenarios) {
        Map<String, Metrics> totals = new LinkedHashMap<>();
        for (var policy : POLICIES) totals.put(policy.name(), new Metrics());
        int[] cameraIndices =
            scenario.reportedRotation() ? new int[] {0, 1, 2, 3} : new int[] {0, 1, 3};
        for (int replayCameraIndex = 0;
            replayCameraIndex < cameraIndices.length;
            replayCameraIndex++) {
          int cameraIndex = cameraIndices[replayCameraIndex];
          Map<String, Metrics> metrics = new LinkedHashMap<>();
          for (var policy : POLICIES) metrics.put(policy.name(), new Metrics());
          var io =
              new VisionIOPhotonVision(
                  "replay-" + scenario.name() + "-" + names[cameraIndex], transforms[cameraIndex]);
          var props = new SimCameraProperties();
          props.setCalibration(
              scenario.reportedRotation() ? 1280 : 800,
              scenario.reportedRotation() ? 800 : 600,
              Rotation2d.fromDegrees(scenario.reportedRotation() ? 70 : 72));
          props.setCalibError(.38, .1);
          try (var cameraSim = new PhotonCameraSim(io.camera, props, APTAG_FIELD_LAYOUT)) {
            cameraSim.enableRawStream(false);
            cameraSim.enableProcessedStream(false);
            cameraSim.enableDrawWireframe(false);
            io.markVisionInitializationComplete(); // Startup policy is tested separately.
            for (long seed : SEEDS) {
              // Preserve the original F/R/L fixture's random sequence, including L's offset of 2.
              props.setRandomSeed(seed + replayCameraIndex);
              for (int i = 0; i < scenario.frames(); i++) {
                double t = i * scenario.dt();
                var truth = scenario.truth(t);
                var captureReference =
                    new Pose2d(
                        truth.toPose2d().getTranslation(),
                        truth
                            .getRotation()
                            .toRotation2d()
                            .plus(Rotation2d.fromDegrees(scenario.headingBias())));
                var generated =
                    cameraSim.process(10, truth.transformBy(transforms[cameraIndex]), targets);
                long captureMicros = 10_000_000 + Math.round(t * 1_000_000);
                var frame =
                    new PhotonPipelineResult(
                        i,
                        captureMicros,
                        captureMicros + 10_000,
                        0,
                        generated.getTargets(),
                        generated.getMultiTagResult());
                cameraSim.submitProcessedFrame(frame); // Publishes the matching calibration.
                io.setHeadingProvider(
                    new VisionIOPhotonVision.VisionHeadingProvider() {
                      public Optional<Rotation2d> getHeadingAtTimestamp(double timestamp) {
                        assertEquals(frame.getTimestampSeconds(), timestamp, 1e-9);
                        return Optional.of(captureReference.getRotation());
                      }

                      public Optional<Pose3d> getSeedPoseAtTimestamp(double timestamp) {
                        return Optional.of(truth);
                      }

                      public double getAngularRateRadPerSec() {
                        return scenario.omega();
                      }

                      public double getLinearSpeedMetersPerSecond() {
                        return scenario.shuttle() ? 3 : 0;
                      }
                    });
                for (var policy : POLICIES) {
                  System.setProperty("vision.photon.strategyOrder", policy.order());
                  if (policy.name().equals("DEFAULT"))
                    System.clearProperty("vision.photon.strategyMode");
                  else
                    System.setProperty(
                        "vision.photon.strategyMode",
                        policy.name().equals("HYBRID") ? "HYBRID" : "STANDARD");
                  var inputs = new VisionIO.VisionIOInputs();
                  io.processResults(List.of(frame), inputs);
                  String sampleKey = scenario.name() + ":" + seed + ":" + i;
                  metrics.get(policy.name()).add(frame, inputs, truth, captureReference, sampleKey);
                  totals.get(policy.name()).add(frame, inputs, truth, captureReference, sampleKey);
                  if (scenario.reportedRotation() && policy.name().equals("MULTI"))
                    reportedRotations.add(frame, inputs, truth, captureReference, sampleKey);
                  frameCsv.add(
                      frameCsv(
                          scenario,
                          policy.name(),
                          names[cameraIndex],
                          cameraIndex,
                          seed,
                          frame,
                          inputs,
                          truth,
                          captureReference));
                  for (var obs : inputs.getPoseObservations()) {
                    if (Math.abs(scenario.omega()) > CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC)
                      assertNotEquals("CONSTRAINED_SOLVEPNP", obs.solver());
                    if (Math.abs(scenario.omega()) > TRIG_MAX_ANGULAR_RATE_RAD_PER_SEC)
                      assertNotEquals("PNP_DISTANCE_TRIG_SOLVE", obs.solver());
                  }
                }
              }
            }
          } finally {
            io.camera.close();
          }
          for (var policy : POLICIES)
            csv.add(
                metrics.get(policy.name()).csv(scenario.name(), policy.name(), names[cameraIndex]));
        }
        for (var policy : POLICIES) {
          var metric = totals.get(policy.name());
          csv.add(metric.csv(scenario.name(), policy.name(), "ALL"));
          System.out.println("REPLAY " + metric.csv(scenario.name(), policy.name(), "ALL"));
          if (metric.targets > 0)
            assertTrue(metric.solved > 0, scenario.name() + " " + policy.name());
        }
        var baseline = totals.get("MULTI");
        var selected = totals.get("DEFAULT");
        var hybrid = totals.get("HYBRID");
        assertEquals(
            hybrid.solvers,
            selected.solvers,
            "Default must execute the validated hybrid policy: " + scenario.name());
        if (scenario.reportedRotation()) {
          assertTrue(selected.max < .15, "Full rotation accepted error: " + selected.max);
          assertTrue(
              selected.acceptedSamples.size() > .94 * scenario.frames() * SEEDS.length,
              "Full rotation needs an accepted camera at over 94% of sample instants");
        } else {
          assertEquals(
              0,
              selected.rejectedAccurate,
              "Heading gate discarded a trench/shuttle observation within 10 cm: "
                  + scenario.name());
          assertTrue(selected.max < .5, "Large retained trench/shuttle error: " + scenario.name());
        }
        if (scenario.name().equals("left_static_90")) {
          assertEquals(0, selected.accepted, "Known coverage gap with back camera disabled");
        } else {
          assertTrue(
              selected.accepted >= FRAMES * SEEDS.length / 2,
              "Scenario must supply useful new observations: " + scenario.name());
        }
        if (baseline.accepted > 0) {
          assertTrue(
              selected.accepted >= .8 * baseline.accepted,
              "Coverage regression: " + scenario.name());
          if (Math.abs(scenario.omega()) <= Math.PI / 2) {
            // Heading error contributes approximately range * angle to translation error.
            // 7 m accepted range plus 0.5 m camera lever arm, with a 4 cm noise allowance.
            double limit = .04 + 7.5 * Math.abs(Math.toRadians(scenario.headingBias()));
            assertTrue(
                selected.rmse() < limit,
                "Solver RMSE regression: " + scenario.name() + " " + selected.rmse());
            assertTrue(
                selected.max < 2 * limit,
                "Large accepted solver error: " + scenario.name() + " " + selected.max);
            assertTrue(
                selected.solvers.getOrDefault("CONSTRAINED_SOLVEPNP", 0) > 0,
                "Eligible-motion comparison must actually exercise constrained PnP");
          } else if (Math.abs(scenario.omega()) > Math.PI / 2) {
            assertEquals(
                baseline.solvers, selected.solvers, "Fast rotation must use non-heading solvers");
            assertEquals(baseline.rmse(), selected.rmse(), 1e-9);
          }
        }
      }
      assertTrue(
          reportedRotations.largeRawCoprocessor > 0,
          "The coprocessor baseline must still reproduce a two-tag error over one meter");
    } finally {
      restore("vision.photon.strategyOrder", oldOrder);
      restore("vision.photon.strategyMode", oldMode);
      Path path = Path.of("build/vision-strategy-revisit/solver-comparison.csv");
      Files.createDirectories(path.getParent());
      Files.write(path, csv);
      Files.write(path.resolveSibling("solver-frames.csv"), frameCsv);
    }
  }

  @ParameterizedTest
  @ValueSource(doubles = {-6, -5.01, -4.99, -2, 2, 4.99, 5.01, 6})
  void headingGateRequiresAlignmentAndCanRejectAnAccuratePoseWhenGyroIsBiased(double bias) {
    assertTrue(HAL.initialize(500, 0));
    String oldOrder = System.getProperty("vision.photon.strategyOrder");
    String oldMode = System.getProperty("vision.photon.strategyMode");
    var io = new VisionIOPhotonVision("quality-heading-bias", APTAG_POSE_EST_CAM_F_POS);
    var props = new SimCameraProperties();
    props.setCalibration(800, 600, Rotation2d.fromDegrees(72));
    // Isolate the gyro bias from image noise so either side of five degrees is meaningful.
    props.setCalibError(0, 0);
    var truth = new Pose3d(4.407, .650, 0, new Rotation3d(0, 0, Math.PI / 2));
    try (var sim = new PhotonCameraSim(io.camera, props, APTAG_FIELD_LAYOUT)) {
      sim.enableRawStream(false);
      sim.enableProcessedStream(false);
      sim.enableDrawWireframe(false);
      var targets =
          APTAG_FIELD_LAYOUT.getTags().stream()
              .map(tag -> new VisionTargetSim(tag.pose, TargetModel.kAprilTag36h11, tag.ID))
              .toList();
      var frame = sim.process(10, truth.transformBy(APTAG_POSE_EST_CAM_F_POS), targets);
      sim.submitProcessedFrame(frame);
      System.setProperty("vision.photon.strategyOrder", "MULTI_TAG_PNP_ON_COPROCESSOR");
      System.setProperty("vision.photon.strategyMode", "STANDARD");
      var inputs = new VisionIO.VisionIOInputs();
      io.processResults(List.of(frame), inputs);
      assertEquals(1, inputs.getPoseObservations().length);
      var observation = inputs.getPoseObservations()[0];
      assertEquals("MULTI_TAG_PNP_ON_COPROCESSOR", observation.solver());
      assertTrue(
          observation
                  .pose()
                  .toPose2d()
                  .getTranslation()
                  .getDistance(truth.toPose2d().getTranslation())
              < .01,
          "The camera pose is accurate even when the independent gyro is biased");
      assertEquals(
          0,
          observation
              .pose()
              .getRotation()
              .toRotation2d()
              .minus(truth.getRotation().toRotation2d())
              .getDegrees(),
          .001);
      var captureReference =
          new Pose2d(
              truth.toPose2d().getTranslation(),
              truth.getRotation().toRotation2d().plus(Rotation2d.fromDegrees(bias)));
      assertTrue(
          VisionSubsystem.rejectionReason(observation, captureReference, false).isEmpty(),
          "An unaligned gyro must not prevent startup qualification");
      assertEquals(
          Math.abs(bias) > 5,
          VisionSubsystem.rejectionReason(observation, captureReference, true).isPresent(),
          "Once aligned, sufficient gyro bias can reject a truly accurate camera measurement");
    } finally {
      io.camera.close();
      restore("vision.photon.strategyOrder", oldOrder);
      restore("vision.photon.strategyMode", oldMode);
    }
  }

  @ParameterizedTest
  @ValueSource(
      doubles = {
        -Math.PI / 2 - .00001,
        -Math.PI / 2,
        -.74,
        .74,
        Math.PI / 2,
        Math.PI / 2 + .00001,
        Double.NaN,
        Double.NEGATIVE_INFINITY,
        Double.POSITIVE_INFINITY
      })
  void constrainedExecutesOnlyInsideItsAngularRateBoundary(double rate) {
    assertTrue(HAL.initialize(500, 0));
    String oldOrder = System.getProperty("vision.photon.strategyOrder");
    String oldMode = System.getProperty("vision.photon.strategyMode");
    var io = new VisionIOPhotonVision("constrained-boundary-" + rate, APTAG_POSE_EST_CAM_F_POS);
    var props = new SimCameraProperties();
    props.setCalibration(800, 600, Rotation2d.fromDegrees(72));
    props.setRandomSeed(2375);
    var truth = new Pose3d(4.72, .59, 0, new Rotation3d(0, 0, Math.PI));
    try (var sim = new PhotonCameraSim(io.camera, props, APTAG_FIELD_LAYOUT)) {
      sim.enableRawStream(false);
      sim.enableProcessedStream(false);
      sim.enableDrawWireframe(false);
      var targets =
          APTAG_FIELD_LAYOUT.getTags().stream()
              .filter(tag -> tag.ID == 29 || tag.ID == 30)
              .map(tag -> new VisionTargetSim(tag.pose, TargetModel.kAprilTag36h11, tag.ID))
              .toList();
      var frame = sim.process(10, truth.transformBy(APTAG_POSE_EST_CAM_F_POS), targets);
      assertEquals(2, frame.getTargets().size(), "Boundary fixture needs parallel tag faces");
      sim.submitProcessedFrame(frame);
      io.markVisionInitializationComplete();
      io.setHeadingProvider(
          new VisionIOPhotonVision.VisionHeadingProvider() {
            public Optional<Rotation2d> getHeadingAtTimestamp(double timestamp) {
              return Optional.of(truth.getRotation().toRotation2d());
            }

            public Optional<Pose3d> getSeedPoseAtTimestamp(double timestamp) {
              return Optional.of(truth);
            }

            public double getAngularRateRadPerSec() {
              return rate;
            }

            public double getLinearSpeedMetersPerSecond() {
              return 0;
            }
          });
      System.setProperty("vision.photon.strategyMode", "HYBRID");
      for (String order :
          List.of("CONSTRAINED_SOLVEPNP,MULTI_TAG_PNP_ON_COPROCESSOR,LOWEST_AMBIGUITY", "")) {
        System.setProperty("vision.photon.strategyOrder", order);
        var inputs = new VisionIO.VisionIOInputs();
        io.processResults(List.of(frame), inputs);
        assertEquals(1, inputs.getPoseObservations().length);
        assertEquals(
            Double.isFinite(rate) && Math.abs(rate) <= Math.PI / 2
                ? "CONSTRAINED_SOLVEPNP"
                : "MULTI_TAG_PNP_ON_COPROCESSOR",
            inputs.getPoseObservations()[0].solver(),
            "Both Hybrid selection and explicit constrained eligibility must respect +/-90 deg/s");
      }
    } finally {
      io.camera.close();
      restore("vision.photon.strategyOrder", oldOrder);
      restore("vision.photon.strategyMode", oldMode);
    }
  }

  private static String frameCsv(
      Scenario scenario,
      String policy,
      String camera,
      int cameraIndex,
      long seed,
      PhotonPipelineResult frame,
      VisionIO.VisionIOInputs inputs,
      Pose3d truth,
      Pose2d captureReference) {
    List<Object> row =
        new ArrayList<>(
            List.of(
                scenario.name(),
                scenario.reportedRotation() ? "1280x800_70deg_4cam" : "800x600_72deg_3cam",
                policy,
                camera,
                seed,
                frame.metadata.sequenceID,
                frame.getTimestampSeconds(),
                scenario.omega(),
                scenario.headingBias(),
                truth.getX(),
                truth.getY(),
                truth.getRotation().getZ(),
                frame.getTargets().size()));
    var observations = inputs.getPoseObservations();
    assertTrue(observations.length <= 1, "Each camera frame may contribute at most one pose");
    if (observations.length == 0) {
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
              Double.NaN,
              false,
              false,
              "NO_POSE"));
    } else {
      var obs = observations[0];
      var rejection = VisionSubsystem.rejectionReason(obs, captureReference, true);
      double headingDifference =
          obs.pose().getRotation().getZ() - captureReference.getRotation().getRadians();
      double yawResidual =
          Math.toDegrees(Math.atan2(Math.sin(headingDifference), Math.cos(headingDifference)));
      row.addAll(
          List.of(
              true,
              obs.solver(),
              obs.tagCount(),
              Arrays.stream(obs.tagIDs())
                  .mapToObj(Integer::toString)
                  .collect(Collectors.joining("|")),
              obs.averageTagDistance(),
              VisionSubsystem.standardDeviations(obs, cameraIndex, false).get(0, 0),
              obs.ambiguity(),
              obs.pose().getX(),
              obs.pose().getY(),
              obs.pose().getZ(),
              obs.pose().getRotation().getZ(),
              yawResidual,
              obs.pose().toPose2d().getTranslation().getDistance(truth.toPose2d().getTranslation()),
              VisionSubsystem.rejectionReason(obs).isEmpty(),
              rejection.isEmpty(),
              rejection.orElse("")));
    }
    row.add(frame.getMultiTagResult().map(m -> m.estimatedPose.bestReprojErr).orElse(Double.NaN));
    row.add(
        Arrays.stream(inputs.getFrameDiagnostics())
            .map(VisionIO.FrameDiagnostic::status)
            .collect(Collectors.joining("|")));
    return row.stream()
        .map(value -> "\"" + value.toString().replace("\"", "\"\"") + "\"")
        .collect(Collectors.joining(","));
  }

  private static void restore(String key, String value) {
    if (value == null) System.clearProperty(key);
    else System.setProperty(key, value);
  }
}
