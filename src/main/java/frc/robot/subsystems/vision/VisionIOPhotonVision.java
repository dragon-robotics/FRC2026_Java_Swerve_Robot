package frc.robot.subsystems.vision;

import static frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT;
import static frc.robot.util.constants.VisionConstants.CONSTRAINED_HEADING_SCALE_FACTOR;
import static frc.robot.util.constants.VisionConstants.CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC;
import static frc.robot.util.constants.VisionConstants.ENABLE_CONSTRAINED_FALLBACK;
import static frc.robot.util.constants.VisionConstants.MAX_TAG_DISTANCE;
import static frc.robot.util.constants.VisionConstants.PHOTON_POSE_STRATEGY_MODE;
import static frc.robot.util.constants.VisionConstants.PHOTON_POSE_STRATEGY_ORDER;
import static frc.robot.util.constants.VisionConstants.TRIG_MAX_ANGULAR_RATE_RAD_PER_SEC;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * PhotonVision camera IO for AprilTag pose estimation.
 *
 * <p>This class owns camera-frame processing only: it reads unread PhotonVision pipeline results,
 * attempts configured pose solvers, and converts successful estimates into {@link PoseObservation}
 * values for {@link VisionSubsystem}. Field-bound checks, drivetrain innovation checks, and
 * standard-deviation weighting stay in the subsystem.
 */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final PhotonPoseEstimator poseEstimator;
  private VisionHeadingProvider headingProvider;

  private static final TargetObservation NO_TARGET =
      new TargetObservation(new Rotation2d(), new Rotation2d());

  // Kept as a quick throttle knob if processing every unread frame gets too expensive.
  @SuppressWarnings("unused")
  private static final int MAX_RESULTS_PER_UPDATE = 2;

  private static final String STRATEGY_MODE_PROPERTY = "vision.photon.strategyMode";
  private static final String TAG_DISTANCE_CONFIDENCE_MODE_PROPERTY =
      "vision.tagDistanceConfidenceMode";
  private static final String HYBRID_STRATEGY_MODE = "HYBRID";
  private static final double HYBRID_TRANSLATION_SPEED_THRESHOLD_METERS_PER_SECOND = 0.5;
  private static final TagDistanceConfidenceMode TAG_DISTANCE_CONFIDENCE_MODE =
      configuredTagDistanceConfidenceMode();

  enum TagDistanceConfidenceMode {
    /** Average every tag used by the pose estimate. */
    ALL_TAG_AVERAGE,
    /** Use the farthest tag used by the pose estimate. */
    MAX_TAG_DISTANCE
  }

  /** Supplies drivetrain state needed by heading-seeded PhotonVision strategies. */
  public interface VisionHeadingProvider {
    /**
     * Returns drivetrain heading at a PhotonVision frame timestamp.
     *
     * @param fpgaTimestampSeconds FPGA timestamp in seconds
     */
    Optional<Rotation2d> getHeadingAtTimestamp(double fpgaTimestampSeconds);

    /**
     * Returns a field-relative pose seed for constrained SolvePnP.
     *
     * @param fpgaTimestampSeconds FPGA timestamp in seconds
     */
    Optional<Pose3d> getSeedPoseAtTimestamp(double fpgaTimestampSeconds);

    /** Returns current drivetrain angular rate in radians per second. */
    double getAngularRateRadPerSec();

    /** Returns current drivetrain linear speed in meters per second. */
    double getLinearSpeedMetersPerSecond();
  }

  // Pre-allocated reusable collections keep per-loop GC pressure low on the
  // roboRIO.
  private final List<PoseObservation> poseObservations = new ArrayList<>(4);
  private final List<FrameDiagnostic> frameDiagnostics = new ArrayList<>(4);
  private int[] tagIdBuffer = new int[16];
  private int tagIdCount = 0;
  private boolean preferMultitagUntilInitialized = true;

  private static final PoseObservation[] EMPTY_POSE_OBSERVATIONS = new PoseObservation[0];
  private static final FrameDiagnostic[] EMPTY_FRAME_DIAGNOSTICS = new FrameDiagnostic[0];
  private static final int[] EMPTY_TAG_IDS = new int[0];

  /**
   * Creates a PhotonVision IO wrapper.
   *
   * @param name PhotonVision camera name
   * @param robotToCamera transform from robot frame to camera frame
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.poseEstimator = new PhotonPoseEstimator(APTAG_FIELD_LAYOUT, robotToCamera);

    String configKey = "Vision/" + name + "/Configuration";
    String requestedOrder = System.getProperty("vision.photon.strategyOrder");
    boolean explicitOrder = requestedOrder != null && !requestedOrder.isBlank();
    boolean hybridMode =
        !explicitOrder
            && HYBRID_STRATEGY_MODE.equalsIgnoreCase(
                System.getProperty(STRATEGY_MODE_PROPERTY, PHOTON_POSE_STRATEGY_MODE));
    DogLog.log(configKey + "/RobotToCamera", robotToCamera);
    DogLog.log(configKey + "/StrategyMode", hybridMode ? HYBRID_STRATEGY_MODE : "STANDARD");
    DogLog.log(
        configKey + "/StrategyOrder",
        hybridMode
            ? "HYBRID_PER_FRAME"
            : Arrays.toString(
                parseStrategyOrder(explicitOrder ? requestedOrder : PHOTON_POSE_STRATEGY_ORDER)));
    DogLog.log(
        configKey + "/StartupStrategyOrder", "MULTI_TAG_PNP_ON_COPROCESSOR,LOWEST_AMBIGUITY");
    DogLog.log(configKey + "/TagDistanceConfidenceMode", TAG_DISTANCE_CONFIDENCE_MODE.name());
  }

  @Override
  public String getCameraName() {
    return camera.getName();
  }

  /** Sets the drivetrain-state source used by heading-seeded pose solvers. */
  public void setHeadingProvider(VisionHeadingProvider headingProvider) {
    this.headingProvider = headingProvider;
  }

  /** Allows normal dynamic strategy selection after stable startup localization completes. */
  public void markVisionInitializationComplete() {
    preferMultitagUntilInitialized = false;
  }

  /** Restores coprocessor-first startup solving when a qualifying startup snapshot expires. */
  public void restartVisionInitialization() {
    preferMultitagUntilInitialized = true;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.setConnected(camera.isConnected());
    inputs.setCameraName(camera.getName());

    processResults(camera.getAllUnreadResults(), inputs);
  }

  // Shared batch boundary for live camera reads and deterministic recorded-frame replay.
  void processResults(List<PhotonPipelineResult> allResults, VisionIOInputs inputs) {
    poseObservations.clear();
    frameDiagnostics.clear();
    tagIdCount = 0;

    if (allResults.isEmpty()) {
      inputs.setPoseObservations(EMPTY_POSE_OBSERVATIONS);
      inputs.setFrameDiagnostics(EMPTY_FRAME_DIAGNOSTICS);
      inputs.setTagIds(EMPTY_TAG_IDS);
      inputs.setLatestTargetObservation(NO_TARGET);
      return;
    }

    for (int resultIndex = 0; resultIndex < allResults.size(); resultIndex++) {
      processResult(allResults.get(resultIndex), inputs);
    }

    inputs.setPoseObservations(
        poseObservations.isEmpty()
            ? EMPTY_POSE_OBSERVATIONS
            : poseObservations.toArray(EMPTY_POSE_OBSERVATIONS));
    inputs.setFrameDiagnostics(frameDiagnostics.toArray(EMPTY_FRAME_DIAGNOSTICS));
    inputs.setTagIds(tagIdCount == 0 ? EMPTY_TAG_IDS : Arrays.copyOf(tagIdBuffer, tagIdCount));
  }

  private void processResult(PhotonPipelineResult result, VisionIOInputs inputs) {
    if (!result.hasTargets()) {
      inputs.setLatestTargetObservation(NO_TARGET);
      recordFrameDiagnostic(result, "NO_TARGETS", "NONE");
      return;
    }

    PhotonTrackedTarget best = result.getBestTarget();
    inputs.setLatestTargetObservation(
        new TargetObservation(
            Rotation2d.fromDegrees(best.getYaw()), Rotation2d.fromDegrees(best.getPitch())));

    List<PhotonTrackedTarget> targets = result.getTargets();
    if (targets.isEmpty() || allTargetsBeyondMaxRange(targets)) {
      recordFrameDiagnostic(result, "ALL_TARGETS_BEYOND_RANGE", "NONE");
      return;
    }

    Optional<EstimatedRobotPose> visionEst = estimateWithConfiguredStrategies(result);

    if (visionEst.isEmpty()) {
      recordFrameDiagnostic(result, "NO_POSE", "NONE");
      return;
    }

    EstimatedRobotPose estimatedPose = visionEst.get();
    Optional<List<PhotonTrackedTarget>> contributingTargets =
        targetsUsedByStrategy(result, estimatedPose.strategy);
    if (contributingTargets.isEmpty()) {
      recordFrameDiagnostic(result, "INVALID_SOLVE_METADATA", estimatedPose.strategy.name());
      return;
    }
    addPoseObservation(estimatedPose, contributingTargets.get(), result.metadata.sequenceID);
    recordFrameDiagnostic(result, "POSE_OBSERVATION", estimatedPose.strategy.name());
  }

  private void recordFrameDiagnostic(PhotonPipelineResult result, String status, String solver) {
    frameDiagnostics.add(
        new FrameDiagnostic(
            result.getTimestampSeconds(),
            result.metadata.sequenceID,
            status,
            solver,
            toObservedTagIds(result.getTargets())));
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
        return estimate;
      }
    }
    return Optional.empty();
  }

  private static int[] toObservedTagIds(List<PhotonTrackedTarget> targets) {
    int[] observedTagIds = new int[targets.size()];
    int observedTagCount = 0;
    for (PhotonTrackedTarget target : targets) {
      int tagId = target.getFiducialId();
      if (tagId <= 0) {
        continue;
      }
      observedTagIds[observedTagCount++] = tagId;
    }
    return observedTagCount == observedTagIds.length
        ? observedTagIds
        : Arrays.copyOf(observedTagIds, observedTagCount);
  }

  /**
   * PhotonLib 2026.3.4 returns every visible target in targetsUsed, including for single-tag
   * solvers. Reconstruct support from the actual strategy, never from that convenience list.
   */
  private static Optional<List<PhotonTrackedTarget>> targetsUsedByStrategy(
      PhotonPipelineResult result, PoseStrategy strategy) {
    List<PhotonTrackedTarget> contributingTargets = new ArrayList<>();
    switch (strategy) {
      case MULTI_TAG_PNP_ON_COPROCESSOR:
        if (result.getMultiTagResult().isEmpty()) {
          return Optional.empty();
        }
        List<Short> contributingIds = result.getMultiTagResult().get().fiducialIDsUsed;
        if (contributingIds == null || contributingIds.size() < 2) {
          return Optional.empty();
        }
        for (short id : contributingIds) {
          if (APTAG_FIELD_LAYOUT.getTagPose(id).isEmpty()
              || contributingTargets.stream().anyMatch(target -> target.getFiducialId() == id)) {
            return Optional.empty();
          }
          PhotonTrackedTarget matchingTarget = null;
          for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == id) {
              if (matchingTarget != null) {
                return Optional.empty();
              }
              matchingTarget = target;
            }
          }
          if (matchingTarget == null) {
            return Optional.empty();
          }
          contributingTargets.add(matchingTarget);
        }
        break;
      case LOWEST_AMBIGUITY:
        // Match PhotonLib exactly: first minimum wins, and only the -1 sentinel is excluded.
        double lowestAmbiguity = 10.0;
        PhotonTrackedTarget selectedTarget = null;
        for (PhotonTrackedTarget target : result.getTargets()) {
          double ambiguity = target.getPoseAmbiguity();
          if (ambiguity != -1 && ambiguity < lowestAmbiguity) {
            lowestAmbiguity = ambiguity;
            selectedTarget = target;
          }
        }
        if (selectedTarget != null) {
          contributingTargets.add(selectedTarget);
        }
        break;
      case PNP_DISTANCE_TRIG_SOLVE:
        if (result.getBestTarget() != null) {
          contributingTargets.add(result.getBestTarget());
        }
        break;
      case CONSTRAINED_SOLVEPNP:
        // VisionEstimation excludes layout-unknown tags before constructing the corner solve.
        for (PhotonTrackedTarget target : result.getTargets()) {
          if (APTAG_FIELD_LAYOUT.getTagPose(target.getFiducialId()).isPresent()) {
            if (target.getDetectedCorners() == null || target.getDetectedCorners().size() != 4) {
              return Optional.empty();
            }
            contributingTargets.add(target);
          }
        }
        break;
      default:
        return Optional.empty();
    }
    if (contributingTargets.isEmpty()) {
      return Optional.empty();
    }
    for (PhotonTrackedTarget target : contributingTargets) {
      if (APTAG_FIELD_LAYOUT.getTagPose(target.getFiducialId()).isEmpty()) {
        return Optional.empty();
      }
    }
    return Optional.of(contributingTargets);
  }

  private PoseStrategy[] resolveStrategyOrder(PhotonPipelineResult result) {
    if (preferMultitagUntilInitialized) {
      // Startup localization must prioritize coprocessor multi-tag until vision
      // has produced a stable initialization sequence.
      return new PoseStrategy[] {
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PoseStrategy.LOWEST_AMBIGUITY
      };
    }

    String configuredStrategyOrder = System.getProperty("vision.photon.strategyOrder");
    if (configuredStrategyOrder != null && !configuredStrategyOrder.isBlank()) {
      return parseStrategyOrder(configuredStrategyOrder);
    }

    if (HYBRID_STRATEGY_MODE.equalsIgnoreCase(
        System.getProperty(STRATEGY_MODE_PROPERTY, PHOTON_POSE_STRATEGY_MODE))) {
      return resolveHybridStrategyOrder(result);
    }

    return parseStrategyOrder(PHOTON_POSE_STRATEGY_ORDER);
  }

  private PoseStrategy[] resolveHybridStrategyOrder(PhotonPipelineResult result) {
    double linearSpeedMetersPerSecond =
        headingProvider == null ? 0.0 : headingProvider.getLinearSpeedMetersPerSecond();
    double angularRateRadPerSec =
        headingProvider == null ? 0.0 : Math.abs(headingProvider.getAngularRateRadPerSec());
    int visibleTargetCount = result.getTargets().size();
    int[] observedTagIds = toObservedTagIds(result.getTargets());
    boolean coplanarTargetSet = haveParallelTagFaces(observedTagIds);

    DogLog.log("Vision/TargetCount", visibleTargetCount);
    DogLog.log("Vision/CoplanarTargetSet", coplanarTargetSet);

    return hybridStrategyOrder(
        visibleTargetCount, coplanarTargetSet, linearSpeedMetersPerSecond, angularRateRadPerSec);
  }

  /** Hybrid strategy classifier; AprilTag face normals point along local +X. */
  private static boolean haveParallelTagFaces(int[] tagIds) {
    if (tagIds.length <= 1) {
      return true;
    }
    Optional<Pose3d> firstTag = APTAG_FIELD_LAYOUT.getTagPose(tagIds[0]);
    if (firstTag.isEmpty()) {
      return true;
    }
    Translation3d tagNormal = new Translation3d(1.0, 0.0, 0.0);
    Translation3d firstNormal = tagNormal.rotateBy(firstTag.get().getRotation());
    double minimumDotProduct = Math.cos(Math.toRadians(15.0));
    for (int tagId : tagIds) {
      Optional<Pose3d> tag = APTAG_FIELD_LAYOUT.getTagPose(tagId);
      if (tag.isEmpty()) {
        continue;
      }
      Translation3d normal = tagNormal.rotateBy(tag.get().getRotation());
      double dotProduct =
          firstNormal.getX() * normal.getX()
              + firstNormal.getY() * normal.getY()
              + firstNormal.getZ() * normal.getZ();
      if (dotProduct < minimumDotProduct) {
        return false;
      }
    }
    return true;
  }

  private static PoseStrategy[] hybridStrategyOrder(
      int visibleTargetCount,
      boolean coplanarTargetSet,
      double linearSpeedMetersPerSecond,
      double angularRateRadPerSec) {
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
      if (coplanarTargetSet) {
        return new PoseStrategy[] {
          PoseStrategy.CONSTRAINED_SOLVEPNP,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
          PoseStrategy.LOWEST_AMBIGUITY
        };
      }

      return new PoseStrategy[] {
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PoseStrategy.CONSTRAINED_SOLVEPNP,
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
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

    if (!Double.isFinite(headingProvider.getAngularRateRadPerSec())
        || Math.abs(headingProvider.getAngularRateRadPerSec())
            > TRIG_MAX_ANGULAR_RATE_RAD_PER_SEC) {
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
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PoseStrategy.LOWEST_AMBIGUITY
      };
    }

    return parsed.toArray(new PoseStrategy[0]);
  }

  private Optional<EstimatedRobotPose> estimateConstrainedFallbackPose(
      PhotonPipelineResult result) {
    if (!ENABLE_CONSTRAINED_FALLBACK || headingProvider == null) {
      return Optional.empty();
    }

    if (!Double.isFinite(headingProvider.getAngularRateRadPerSec())
        || Math.abs(headingProvider.getAngularRateRadPerSec())
            > CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC) {
      return Optional.empty();
    }

    Optional<Rotation2d> headingSample =
        headingProvider.getHeadingAtTimestamp(result.getTimestampSeconds());
    if (headingSample.isEmpty()) {
      return Optional.empty();
    }

    // A valid coprocessor solution is a better starting point than an ambiguous single tag.
    Optional<EstimatedRobotPose> seedEstimate = poseEstimator.estimateCoprocMultiTagPose(result);
    if (seedEstimate.isEmpty()) seedEstimate = poseEstimator.estimateLowestAmbiguityPose(result);
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
      EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets, long frameSequenceId) {
    int[] observedTagIds = new int[targets.size()];
    int observedTagCount = 0;
    int distanceSampleCountAll = 0;
    double totalDistanceAll = 0.0;
    double maxDistanceAll = 0.0;
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
        double distanceMeters = cameraToTarget.getTranslation().getNorm();
        totalDistanceAll += distanceMeters;
        distanceSampleCountAll++;
        maxDistanceAll = Math.max(maxDistanceAll, distanceMeters);
      }

      totalAmbiguity += target.getPoseAmbiguity();
    }

    if (observedTagCount == 0) {
      return;
    }

    if (observedTagCount < observedTagIds.length) {
      observedTagIds = Arrays.copyOf(observedTagIds, observedTagCount);
    }

    VisionIO.PoseObservationType observationType =
        estimatedPose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
            ? VisionIO.PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR
            : VisionIO.PoseObservationType.PHOTONVISION;

    double averageTagDistanceMeters =
        confidenceDistance(
            TAG_DISTANCE_CONFIDENCE_MODE, totalDistanceAll, distanceSampleCountAll, maxDistanceAll);

    poseObservations.add(
        new PoseObservation(
            estimatedPose.timestampSeconds,
            estimatedPose.estimatedPose,
            totalAmbiguity / observedTagCount,
            observedTagCount,
            averageTagDistanceMeters,
            observationType,
            observedTagIds,
            estimatedPose.strategy.name(),
            frameSequenceId));
  }

  static double confidenceDistanceForTest(
      TagDistanceConfidenceMode mode,
      double totalDistanceAll,
      int distanceSampleCountAll,
      double maxDistanceAll) {
    return confidenceDistance(mode, totalDistanceAll, distanceSampleCountAll, maxDistanceAll);
  }

  private static double confidenceDistance(
      TagDistanceConfidenceMode mode,
      double totalDistanceAll,
      int distanceSampleCountAll,
      double maxDistanceAll) {
    if (distanceSampleCountAll <= 0) {
      // No usable distance samples; force the observation to fail the distance gate.
      return Double.POSITIVE_INFINITY;
    }

    return switch (mode) {
      case ALL_TAG_AVERAGE -> totalDistanceAll / distanceSampleCountAll;
      case MAX_TAG_DISTANCE -> maxDistanceAll;
    };
  }

  private static TagDistanceConfidenceMode configuredTagDistanceConfidenceMode() {
    String raw =
        System.getProperty(
            TAG_DISTANCE_CONFIDENCE_MODE_PROPERTY,
            TagDistanceConfidenceMode.ALL_TAG_AVERAGE.name());
    try {
      return TagDistanceConfidenceMode.valueOf(raw.trim().toUpperCase(Locale.ROOT));
    } catch (IllegalArgumentException ex) {
      return TagDistanceConfidenceMode.ALL_TAG_AVERAGE;
    }
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
