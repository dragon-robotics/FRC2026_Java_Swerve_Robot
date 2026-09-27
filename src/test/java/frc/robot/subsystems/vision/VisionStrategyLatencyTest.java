package frc.robot.subsystems.vision;

import static frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT;
import static frc.robot.util.constants.VisionConstants.APTAG_POSE_EST_CAM_F_POS;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

/** Emulates delayed capture heading with the current angular rate supplied by production. */
class VisionStrategyLatencyTest {
  @ParameterizedTest
  @ValueSource(ints = {-1, 1})
  void processingRateControlsDelayedFramesAcrossBothThresholdCrossings(int direction)
      throws Exception {
    assertTrue(HAL.initialize(500, 0));
    String oldOrder = System.getProperty("vision.photon.strategyOrder");
    String oldMode = System.getProperty("vision.photon.strategyMode");
    var io = new VisionIOPhotonVision("latency-crossing-" + direction, APTAG_POSE_EST_CAM_F_POS);
    var properties = new SimCameraProperties();
    properties.setCalibration(800, 600, Rotation2d.fromDegrees(72));
    properties.setCalibError(0, 0);
    var rows =
        new ArrayList<>(
            List.of(
                "policy,direction,frame,captureSeconds,processingSeconds,captureRateRadPerSec,accelerationRadPerSecSquared,processingRateRadPerSec,captureTruthX,captureTruthY,captureTruthYawRad,processingTruthYawRad,solver,xyErrorM"));
    try (var simulation = new PhotonCameraSim(io.camera, properties, APTAG_FIELD_LAYOUT)) {
      simulation.enableRawStream(false);
      simulation.enableProcessedStream(false);
      simulation.enableDrawWireframe(false);
      var targets =
          APTAG_FIELD_LAYOUT.getTags().stream()
              .filter(tag -> tag.ID == 29 || tag.ID == 30)
              .map(tag -> new VisionTargetSim(tag.pose, TargetModel.kAprilTag36h11, tag.ID))
              .toList();
      System.setProperty("vision.photon.strategyMode", "HYBRID");
      io.markVisionInitializationComplete();

      double captureYaw = Math.PI;
      double[] captureMagnitudes = {Math.PI / 2 - .04, Math.PI / 2 + .04};
      for (int index = 0; index < captureMagnitudes.length; index++) {
        double delay = .04;
        double captureSeconds = 10 + index * delay;
        double captureRate = direction * captureMagnitudes[index];
        double acceleration = direction * (index == 0 ? 2 : -2);
        double processingRate = captureRate + acceleration * delay;
        double processingYaw = captureYaw + captureRate * delay + .5 * acceleration * delay * delay;
        var captureTruth = new Pose3d(4.72, .59, 0, new Rotation3d(0, 0, captureYaw));
        var generated =
            simulation.process(40, captureTruth.transformBy(APTAG_POSE_EST_CAM_F_POS), targets);
        assertEquals(2, generated.getTargets().size(), "Both parallel tags must remain visible");
        long captureMicros = Math.round(captureSeconds * 1_000_000);
        var frame =
            new PhotonPipelineResult(
                index,
                captureMicros,
                captureMicros + 40_000,
                0,
                generated.getTargets(),
                generated.getMultiTagResult());
        simulation.submitProcessedFrame(frame);
        io.setHeadingProvider(
            new VisionIOPhotonVision.VisionHeadingProvider() {
              public Optional<Rotation2d> getHeadingAtTimestamp(double timestamp) {
                assertEquals(captureSeconds, timestamp, 1e-9, "Heading must use capture time");
                return Optional.of(captureTruth.getRotation().toRotation2d());
              }

              public Optional<Pose3d> getSeedPoseAtTimestamp(double timestamp) {
                assertEquals(captureSeconds, timestamp, 1e-9, "Pose seed must use capture time");
                return Optional.empty();
              }

              public double getAngularRateRadPerSec() {
                return processingRate;
              }

              public double getLinearSpeedMetersPerSecond() {
                return 0;
              }
            });
        for (String policy : List.of("EXPLICIT", "HYBRID")) {
          System.setProperty(
              "vision.photon.strategyOrder",
              policy.equals("EXPLICIT")
                  ? "CONSTRAINED_SOLVEPNP,MULTI_TAG_PNP_ON_COPROCESSOR,LOWEST_AMBIGUITY"
                  : "");
          var inputs = new VisionIO.VisionIOInputs();
          io.processResults(List.of(frame), inputs);
          assertEquals(1, inputs.getPoseObservations().length);
          var observation = inputs.getPoseObservations()[0];
          double error =
              observation
                  .pose()
                  .toPose2d()
                  .getTranslation()
                  .getDistance(captureTruth.toPose2d().getTranslation());
          rows.add(
              String.format(
                  Locale.ROOT,
                  "%s,%d,%d,%.6f,%.6f,%.9f,%.6f,%.9f,%.6f,%.6f,%.9f,%.9f,%s,%.9f",
                  policy,
                  direction,
                  index,
                  captureSeconds,
                  captureSeconds + delay,
                  captureRate,
                  acceleration,
                  processingRate,
                  captureTruth.getX(),
                  captureTruth.getY(),
                  captureYaw,
                  processingYaw,
                  observation.solver(),
                  error));
          assertEquals(captureSeconds, observation.timestamp(), 1e-9);
          assertEquals(index, observation.frameSequenceId());
          // First frame crosses above 90 deg/s during latency; the next crosses back below.
          assertEquals(
              index == 0 ? "MULTI_TAG_PNP_ON_COPROCESSOR" : "CONSTRAINED_SOLVEPNP",
              observation.solver(),
              "Current-rate eligibility must differ from the opposite capture-rate decision");
          if (index == 1) {
            assertTrue(error < .05, "Constrained solving must retain the correct delayed heading");
          }
        }
        captureYaw = processingYaw;
      }
    } finally {
      io.camera.close();
      restore("vision.photon.strategyOrder", oldOrder);
      restore("vision.photon.strategyMode", oldMode);
      Path output = Path.of("build/vision-strategy-latency/crossing-" + direction + ".csv");
      Files.createDirectories(output.getParent());
      Files.write(output, rows);
    }
  }

  private static void restore(String name, String value) {
    if (value == null) System.clearProperty(name);
    else System.setProperty(name, value);
  }
}
