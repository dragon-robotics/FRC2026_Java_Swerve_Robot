package frc.robot.subsystems.vision;

import static frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT;
import static frc.robot.util.constants.VisionConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import java.nio.file.*;
import java.util.*;
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
      String name, double y, double yaw, double omega, boolean shuttle, double headingBias) {
    Scenario(String name, double y, double yaw, double omega, boolean shuttle) {
      this(name, y, yaw, omega, shuttle, 0);
    }

    Pose3d truth(double t) {
      return new Pose3d(
          shuttle ? 2.5 : 4.407,
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
    int frames, targets, solved, accepted;
    double sumSquared, max, rawSumSquared, rawMax;
    final Map<String, Integer> solvers = new TreeMap<>();
    final Map<String, Integer> rejections = new TreeMap<>();
    final List<Double> errors = new ArrayList<>();

    void add(PhotonPipelineResult frame, VisionIO.VisionIOInputs inputs, Pose3d truth) {
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
        var rejection = VisionSubsystem.rejectionReason(obs);
        if (rejection.isPresent()) {
          rejections.merge(rejection.get().split("=")[0], 1, Integer::sum);
        } else {
          assertTrue(Double.isFinite(error));
          accepted++;
          sumSquared += error * error;
          max = Math.max(max, error);
          errors.add(error);
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
          "%s,%s,%s,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,\"%s\",\"%s\"",
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
          rejections);
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
                "scenario,policy,camera,frames,framesWithTargets,solved,qualityAccepted,acceptedRmseM,acceptedP95M,acceptedMaxM,allSolvedRmseM,allSolvedMaxM,actualSolvers,rejections"));
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
    var targets =
        APTAG_FIELD_LAYOUT.getTags().stream()
            .map(tag -> new VisionTargetSim(tag.pose, TargetModel.kAprilTag36h11, tag.ID))
            .toList();
    Transform3d[] transforms = {
      APTAG_POSE_EST_CAM_F_POS, APTAG_POSE_EST_CAM_R_POS, APTAG_POSE_EST_CAM_L_POS
    };
    String[] names = {"F", "R", "L"};
    try {
      for (var scenario : scenarios) {
        Map<String, Metrics> totals = new LinkedHashMap<>();
        for (var policy : POLICIES) totals.put(policy.name(), new Metrics());
        for (int cameraIndex = 0; cameraIndex < transforms.length; cameraIndex++) {
          Map<String, Metrics> metrics = new LinkedHashMap<>();
          for (var policy : POLICIES) metrics.put(policy.name(), new Metrics());
          var io =
              new VisionIOPhotonVision(
                  "replay-" + scenario.name() + "-" + names[cameraIndex], transforms[cameraIndex]);
          var props = new SimCameraProperties();
          props.setCalibration(800, 600, Rotation2d.fromDegrees(72));
          props.setCalibError(.38, .1);
          try (var cameraSim = new PhotonCameraSim(io.camera, props, APTAG_FIELD_LAYOUT)) {
            cameraSim.enableRawStream(false);
            cameraSim.enableProcessedStream(false);
            cameraSim.enableDrawWireframe(false);
            io.markVisionInitializationComplete(); // Startup policy is tested separately.
            for (long seed : SEEDS) {
              props.setRandomSeed(seed + cameraIndex);
              for (int i = 0; i < FRAMES; i++) {
                double t = i * .05;
                var truth = scenario.truth(t);
                var generated =
                    cameraSim.process(10, truth.transformBy(transforms[cameraIndex]), targets);
                var frame =
                    new PhotonPipelineResult(
                        i,
                        10_000_000 + i * 50_000,
                        10_010_000 + i * 50_000,
                        0,
                        generated.getTargets(),
                        generated.getMultiTagResult());
                cameraSim.submitProcessedFrame(frame); // Publishes the matching calibration.
                io.setHeadingProvider(
                    new VisionIOPhotonVision.VisionHeadingProvider() {
                      public Optional<Rotation2d> getHeadingAtTimestamp(double timestamp) {
                        assertEquals(frame.getTimestampSeconds(), timestamp, 1e-9);
                        return Optional.of(
                            truth
                                .getRotation()
                                .toRotation2d()
                                .plus(Rotation2d.fromDegrees(scenario.headingBias())));
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
                  metrics.get(policy.name()).add(frame, inputs, truth);
                  totals.get(policy.name()).add(frame, inputs, truth);
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
          if (Math.abs(scenario.omega()) <= .5) {
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
                "Slow-motion comparison must actually exercise constrained PnP");
          } else if (Math.abs(scenario.omega()) > 1) {
            assertEquals(
                baseline.solvers, selected.solvers, "Fast rotation must use non-heading solvers");
            assertEquals(baseline.rmse(), selected.rmse(), 1e-9);
          }
        }
      }
    } finally {
      restore("vision.photon.strategyOrder", oldOrder);
      restore("vision.photon.strategyMode", oldMode);
      Path path = Path.of("build/vision-strategy-revisit/solver-comparison.csv");
      Files.createDirectories(path.getParent());
      Files.write(path, csv);
    }
  }

  @ParameterizedTest
  @ValueSource(doubles = {-.50001, -.5, .5, .50001})
  void constrainedExecutesOnlyInsideItsAngularRateBoundary(double rate) {
    assertTrue(HAL.initialize(500, 0));
    String oldOrder = System.getProperty("vision.photon.strategyOrder");
    var io = new VisionIOPhotonVision("constrained-boundary", APTAG_POSE_EST_CAM_F_POS);
    var props = new SimCameraProperties();
    props.setCalibration(800, 600, Rotation2d.fromDegrees(72));
    props.setRandomSeed(2375);
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
      System.setProperty(
          "vision.photon.strategyOrder",
          "CONSTRAINED_SOLVEPNP,MULTI_TAG_PNP_ON_COPROCESSOR,LOWEST_AMBIGUITY");
      var inputs = new VisionIO.VisionIOInputs();
      io.processResults(List.of(frame), inputs);
      assertEquals(1, inputs.getPoseObservations().length);
      assertEquals(
          Math.abs(rate) <= .5 ? "CONSTRAINED_SOLVEPNP" : "MULTI_TAG_PNP_ON_COPROCESSOR",
          inputs.getPoseObservations()[0].solver());
    } finally {
      io.camera.close();
      restore("vision.photon.strategyOrder", oldOrder);
    }
  }

  private static void restore(String key, String value) {
    if (value == null) System.clearProperty(key);
    else System.setProperty(key, value);
  }
}
