package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.json.simple.JSONObject;

/** Atomic, versioned WPILOG string records; no cross-topic alignment is needed for analysis. */
final class VisionDiagnostics {
  private VisionDiagnostics() {}

  record Observation(
      long sequence,
      String camera,
      VisionIO.PoseObservation observation,
      double receivedTimestamp,
      double ctreTimestamp,
      Pose2d reference,
      double innovation,
      Matrix<N3, N1> standardDeviations,
      boolean accepted,
      String rejectionReason,
      boolean startupInnovationBypassAllowed,
      Pose2d fusedBefore,
      Pose2d fusedAfter,
      ChassisSpeeds speeds,
      double pitchDegrees,
      double rollDegrees,
      boolean enabled,
      boolean autonomous,
      int cameraStartupCount,
      String initializationCamera,
      boolean aiming) {
    String toJson() {
      Map<String, Object> data = base("OBSERVATION", sequence, camera, receivedTimestamp);
      data.put("captureTimestampSeconds", finite(observation.timestamp()));
      data.put("ctreTimestampSeconds", finite(ctreTimestamp));
      data.put("frameAgeSeconds", finite(receivedTimestamp - observation.timestamp()));
      data.put("frameSequenceId", observation.frameSequenceId());
      data.put("solver", observation.solver());
      data.put("source", observation.type().name());
      List<Integer> ids = new ArrayList<>();
      for (int id : observation.tagIDs()) ids.add(id);
      data.put("tagIds", ids);
      data.put("tagCount", observation.tagCount());
      data.put("distanceMeters", finite(observation.averageTagDistance()));
      data.put("ambiguity", finite(observation.ambiguity()));
      data.put("rawPose", pose(observation.pose()));
      data.put("referencePose", pose(reference));
      data.put("innovationMeters", finite(innovation));
      data.put(
          "innovationXYMeters",
          reference == null
              ? null
              : numbers(
                  observation.pose().getX() - reference.getX(),
                  observation.pose().getY() - reference.getY()));
      List<Object> sigma =
          numbers(
              standardDeviations.get(0, 0),
              standardDeviations.get(1, 0),
              standardDeviations.get(2, 0));
      data.put("calculatedStdDevs", sigma);
      data.put("suppliedStdDevs", accepted ? sigma : null);
      data.put("accepted", accepted);
      data.put("rejectionReason", rejectionReason);
      data.put("startupInnovationBypassAllowed", startupInnovationBypassAllowed);
      data.put("fusedPoseBefore", pose(fusedBefore));
      data.put("fusedPoseAfter", pose(fusedAfter));
      data.put(
          "speeds",
          numbers(
              speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond));
      data.put("pitchDegrees", finite(pitchDegrees));
      data.put("rollDegrees", finite(rollDegrees));
      data.put("enabled", enabled);
      data.put("autonomous", autonomous);
      data.put("cameraStartupCount", cameraStartupCount);
      data.put("initializationCamera", initializationCamera);
      data.put("aiming", aiming);
      return JSONObject.toJSONString(data);
    }
  }

  static String frame(long sequence, String camera, VisionIO.FrameDiagnostic frame, double now) {
    Map<String, Object> data = base("FRAME", sequence, camera, now);
    data.put("captureTimestampSeconds", finite(frame.timestamp()));
    data.put("frameAgeSeconds", finite(now - frame.timestamp()));
    data.put("frameSequenceId", frame.frameSequenceId());
    data.put("status", frame.status());
    data.put("solver", frame.solver());
    List<Integer> ids = new ArrayList<>();
    for (int id : frame.visibleTagIDs()) ids.add(id);
    data.put("visibleTagIds", ids);
    return JSONObject.toJSONString(data);
  }

  private static Map<String, Object> base(String event, long sequence, String camera, double now) {
    Map<String, Object> data = new LinkedHashMap<>();
    data.put("schemaVersion", 1);
    data.put("event", event);
    data.put("sequence", sequence);
    data.put("camera", camera);
    data.put("processingTimestampSeconds", finite(now));
    return data;
  }

  private static List<Object> pose(Pose2d pose) {
    return pose == null ? null : numbers(pose.getX(), pose.getY(), pose.getRotation().getRadians());
  }

  private static List<Object> pose(Pose3d pose) {
    return numbers(
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        pose.getRotation().getX(),
        pose.getRotation().getY(),
        pose.getRotation().getZ());
  }

  private static List<Object> numbers(double... values) {
    List<Object> result = new ArrayList<>(values.length);
    for (double value : values) result.add(finite(value));
    return result;
  }

  private static Double finite(double value) {
    return Double.isFinite(value) ? value : null;
  }
}
