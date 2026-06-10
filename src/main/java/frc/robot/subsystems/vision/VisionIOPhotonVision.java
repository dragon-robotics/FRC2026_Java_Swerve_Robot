package frc.robot.subsystems.vision;

import static frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT;
import static frc.robot.util.constants.VisionConstants.CONSTRAINED_HEADING_SCALE_FACTOR;
import static frc.robot.util.constants.VisionConstants.CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC;
import static frc.robot.util.constants.VisionConstants.ENABLE_CONSTRAINED_FALLBACK;
import static frc.robot.util.constants.VisionConstants.MAX_TAG_DISTANCE;
import static frc.robot.util.constants.VisionConstants.PHOTON_POSE_STRATEGY_ORDER;
import static frc.robot.util.constants.VisionConstants.TRIG_MAX_ANGULAR_RATE_RAD_PER_SEC;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final PhotonPoseEstimator poseEstimator;
  private VisionHeadingProvider headingProvider;

  private static final TargetObservation NO_TARGET =
      new TargetObservation(new Rotation2d(), new Rotation2d());

  private static final int MAX_RESULTS_PER_UPDATE = 2;
  private static final String STRATEGY_MODE_PROPERTY = "vision.photon.strategyMode";
  private static final String HYBRID_STRATEGY_MODE = "HYBRID";
  private static final double HYBRID_TRANSLATION_SPEED_THRESHOLD_METERS_PER_SECOND = 0.5;

  public interface VisionHeadingProvider {
    Optional<Rotation2d> getHeadingAtTimestamp(double fpgaTimestampSeconds);

    Optional<Pose3d> getSeedPoseAtTimestamp(double fpgaTimestampSeconds);

    double getAngularRateRadPerSec();

    double getLinearSpeedMetersPerSecond();
  }

  // Pre-allocated reusable collections keep per-loop GC pressure low on the
  // roboRIO.
  private final List<PoseObservation> poseObservations = new ArrayList<>(4);
  private int[] tagIdBuffer = new int[16];
  private int tagIdCount = 0;

  private static final PoseObservation[] EMPTY_POSE_OBSERVATIONS = new PoseObservation[0];
  private static final int[] EMPTY_TAG_IDS = new int[0];

  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.poseEstimator = new PhotonPoseEstimator(APTAG_FIELD_LAYOUT, robotToCamera);
  }

  @Override
  public String getCameraName() {
    return camera.getName();
  }

  public void setHeadingProvider(VisionHeadingProvider headingProvider) {
    this.headingProvider = headingProvider;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.setConnected(camera.isConnected());
    inputs.setCameraName(camera.getName());

    poseObservations.clear();
    tagIdCount = 0;

    var allResults = camera.getAllUnreadResults();
    if (allResults.isEmpty()) {
      inputs.setPoseObservations(EMPTY_POSE_OBSERVATIONS);
      inputs.setTagIds(EMPTY_TAG_IDS);
      inputs.setLatestTargetObservation(NO_TARGET);
      return;
    }

    int startIndex = Math.max(0, allResults.size() - MAX_RESULTS_PER_UPDATE);
    for (int resultIndex = startIndex; resultIndex < allResults.size(); resultIndex++) {
      processResult(allResults.get(resultIndex), inputs);
    }

    inputs.setPoseObservations(
        poseObservations.isEmpty()
            ? EMPTY_POSE_OBSERVATIONS
            : poseObservations.toArray(EMPTY_POSE_OBSERVATIONS));
    inputs.setTagIds(tagIdCount == 0 ? EMPTY_TAG_IDS : Arrays.copyOf(tagIdBuffer, tagIdCount));
  }

  private void processResult(PhotonPipelineResult result, VisionIOInputs inputs) {
    if (!result.hasTargets()) {
      inputs.setLatestTargetObservation(NO_TARGET);
      return;
    }

    PhotonTrackedTarget best = result.getBestTarget();
    inputs.setLatestTargetObservation(
        new TargetObservation(
            Rotation2d.fromDegrees(best.getYaw()), Rotation2d.fromDegrees(best.getPitch())));

    List<PhotonTrackedTarget> targets = result.getTargets();
    if (targets.isEmpty() || allTargetsBeyondMaxRange(targets)) {
      return;
    }

    Optional<EstimatedRobotPose> visionEst = estimateWithConfiguredStrategies(result);

    visionEst.ifPresent(
        estimatedPose -> addPoseObservation(estimatedPose, estimatedPose.targetsUsed));
  }

  private Optional<EstimatedRobotPose> estimateWithConfiguredStrategies(
      PhotonPipelineResult result) {
    for (PoseStrategy strategy : resolveStrategyOrder(result)) {
      Optional<EstimatedRobotPose> estimate;
      switch (strategy) {
        case MULTI_TAG_PNP_ON_COPROCESSOR:
          estimate = poseEstimator.estimateCoprocMultiTagPose(result);
          break;
        case CONSTRAINED_SOLVEPNP:
          estimate = estimateConstrainedFallbackPose(result);
          break;
        case PNP_DISTANCE_TRIG_SOLVE:
          estimate = estimatePnpDistanceTrigSolvePose(result);
          break;
        case LOWEST_AMBIGUITY:
          estimate = poseEstimator.estimateLowestAmbiguityPose(result);
          break;
        default:
          estimate = Optional.empty();
          break;
      }
      if (estimate.isPresent()) {
        DogLog.log("Vision/ActivePoseStrategy", strategy.name());
        return estimate;
      }
    }
    DogLog.log("Vision/ActivePoseStrategy", "NONE");
    return Optional.empty();
  }

  private PoseStrategy[] resolveStrategyOrder(PhotonPipelineResult result) {
    if (HYBRID_STRATEGY_MODE.equalsIgnoreCase(System.getProperty(STRATEGY_MODE_PROPERTY, ""))) {
      return resolveHybridStrategyOrder(result);
    }

    return parseStrategyOrder(
        System.getProperty("vision.photon.strategyOrder", PHOTON_POSE_STRATEGY_ORDER));
  }

  private PoseStrategy[] resolveHybridStrategyOrder(PhotonPipelineResult result) {
    double linearSpeedMetersPerSecond =
        headingProvider == null ? 0.0 : headingProvider.getLinearSpeedMetersPerSecond();
    double angularRateRadPerSec =
        headingProvider == null ? 0.0 : Math.abs(headingProvider.getAngularRateRadPerSec());
    int visibleTargetCount = result.getTargets().size();

    DogLog.log("Vision/TargetCount", visibleTargetCount);

    return hybridStrategyOrderForTest(
        visibleTargetCount, linearSpeedMetersPerSecond, angularRateRadPerSec);
  }

  static PoseStrategy[] hybridStrategyOrderForTest(
      int visibleTargetCount, double linearSpeedMetersPerSecond, double angularRateRadPerSec) {
    if (angularRateRadPerSec > CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC) {
      if (visibleTargetCount >= 2) {
        return new PoseStrategy[] {
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
          PoseStrategy.LOWEST_AMBIGUITY
        };
      }

      return new PoseStrategy[] {
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PoseStrategy.LOWEST_AMBIGUITY
      };
    }

    if (visibleTargetCount >= 2) {
      return new PoseStrategy[] {
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        PoseStrategy.CONSTRAINED_SOLVEPNP,
        PoseStrategy.LOWEST_AMBIGUITY
      };
    }

    if (linearSpeedMetersPerSecond > HYBRID_TRANSLATION_SPEED_THRESHOLD_METERS_PER_SECOND) {
      return new PoseStrategy[] {
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PoseStrategy.CONSTRAINED_SOLVEPNP,
        PoseStrategy.LOWEST_AMBIGUITY
      };
    }

    return new PoseStrategy[] {
      PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
      PoseStrategy.CONSTRAINED_SOLVEPNP,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      PoseStrategy.LOWEST_AMBIGUITY
    };
  }

  private Optional<EstimatedRobotPose> estimatePnpDistanceTrigSolvePose(
      PhotonPipelineResult result) {
    if (headingProvider == null) {
      return Optional.empty();
    }

    if (Math.abs(headingProvider.getAngularRateRadPerSec()) > TRIG_MAX_ANGULAR_RATE_RAD_PER_SEC) {
      return Optional.empty();
    }

    Optional<Rotation2d> headingSample =
        headingProvider.getHeadingAtTimestamp(result.getTimestampSeconds());
    if (headingSample.isEmpty()) {
      return Optional.empty();
    }

    poseEstimator.addHeadingData(result.getTimestampSeconds(), headingSample.get());
    return poseEstimator.estimatePnpDistanceTrigSolvePose(result);
  }

  private static PoseStrategy[] parseStrategyOrder(String rawOrder) {
    List<PoseStrategy> parsed = new ArrayList<>();
    for (String token : rawOrder.split(",")) {
      String candidate = token.trim();
      if (candidate.isEmpty()) {
        continue;
      }
      try {
        PoseStrategy strategy = PoseStrategy.valueOf(candidate);
        if (strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
            || strategy == PoseStrategy.CONSTRAINED_SOLVEPNP
            || strategy == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
            || strategy == PoseStrategy.LOWEST_AMBIGUITY) {
          parsed.add(strategy);
        }
      } catch (IllegalArgumentException ignored) {
        // Ignore unknown strategy names from the property string.
      }
    }

    if (parsed.isEmpty()) {
      return new PoseStrategy[] {
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PoseStrategy.CONSTRAINED_SOLVEPNP,
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        PoseStrategy.LOWEST_AMBIGUITY
      };
    }

    return parsed.toArray(new PoseStrategy[0]);
  }

  private Optional<EstimatedRobotPose> estimateConstrainedFallbackPose(
      PhotonPipelineResult result) {
    if (!ENABLE_CONSTRAINED_FALLBACK || headingProvider == null) {
      return Optional.empty();
    }

    if (Math.abs(headingProvider.getAngularRateRadPerSec())
        > CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC) {
      return Optional.empty();
    }

    Optional<Rotation2d> headingSample =
        headingProvider.getHeadingAtTimestamp(result.getTimestampSeconds());
    if (headingSample.isEmpty()) {
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> seedEstimate = poseEstimator.estimateLowestAmbiguityPose(result);
    Optional<Pose3d> seedPose = seedEstimate.map(estimate -> estimate.estimatedPose);
    if (seedPose.isEmpty()) {
      seedPose = headingProvider.getSeedPoseAtTimestamp(result.getTimestampSeconds());
    }
    if (seedPose.isEmpty()) {
      return Optional.empty();
    }

    Optional<Matrix<N3, N3>> cameraMatrix = camera.getCameraMatrix();
    Optional<Matrix<N8, N1>> distCoeffs = camera.getDistCoeffs();
    if (cameraMatrix.isEmpty() || distCoeffs.isEmpty()) {
      return Optional.empty();
    }

    poseEstimator.addHeadingData(result.getTimestampSeconds(), headingSample.get());

    return poseEstimator.estimateConstrainedSolvepnpPose(
        result,
        cameraMatrix.get(),
        distCoeffs.get(),
        seedPose.get(),
        false,
        CONSTRAINED_HEADING_SCALE_FACTOR);
  }

  private boolean allTargetsBeyondMaxRange(List<PhotonTrackedTarget> targets) {
    for (PhotonTrackedTarget target : targets) {
      Transform3d cameraToTarget = target.getBestCameraToTarget();
      if (cameraToTarget != null && cameraToTarget.getTranslation().getNorm() <= MAX_TAG_DISTANCE) {
        return false;
      }
    }
    return true;
  }

  private void addPoseObservation(
      EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
    int[] observedTagIds = new int[targets.size()];
    int observedTagCount = 0;
    int distanceSampleCount = 0;
    double totalDistance = 0.0;
    double totalAmbiguity = 0.0;

    for (PhotonTrackedTarget target : targets) {
      int tagId = target.getFiducialId();
      if (tagId <= 0) {
        continue;
      }

      observedTagIds[observedTagCount++] = tagId;
      addTagId(tagId);

      Transform3d cameraToTarget = target.getBestCameraToTarget();
      if (cameraToTarget != null) {
        totalDistance += cameraToTarget.getTranslation().getNorm();
        distanceSampleCount++;
      }

      totalAmbiguity += Math.max(0.0, target.getPoseAmbiguity());
    }

    if (observedTagCount == 0) {
      return;
    }

    if (observedTagCount < observedTagIds.length) {
      observedTagIds = Arrays.copyOf(observedTagIds, observedTagCount);
    }

    poseObservations.add(
        new PoseObservation(
            estimatedPose.timestampSeconds,
            estimatedPose.estimatedPose,
            totalAmbiguity / observedTagCount,
            observedTagCount,
            distanceSampleCount == 0 ? 0.0 : totalDistance / distanceSampleCount,
            PoseObservationType.PHOTONVISION,
            observedTagIds));
  }

  private void addTagId(int tagId) {
    for (int i = 0; i < tagIdCount; i++) {
      if (tagIdBuffer[i] == tagId) {
        return;
      }
    }

    if (tagIdCount >= tagIdBuffer.length) {
      tagIdBuffer = Arrays.copyOf(tagIdBuffer, tagIdBuffer.length * 2);
    }
    tagIdBuffer[tagIdCount++] = tagId;
  }
}
