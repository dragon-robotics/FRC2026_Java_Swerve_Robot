// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT;
import static frc.robot.util.constants.VisionConstants.APTAG_CAMERA_NAMES;
import static frc.robot.util.constants.VisionConstants.CAMERA_STDDEV_FACTORS;
import static frc.robot.util.constants.VisionConstants.DISABLED_AUTO_RESEED_DELTA_METERS;
import static frc.robot.util.constants.VisionConstants.DISABLED_AUTO_RESEED_MIN_INTERVAL_SECONDS;
import static frc.robot.util.constants.VisionConstants.DISABLED_AUTO_RESEED_MIN_TAG_COUNT;
import static frc.robot.util.constants.VisionConstants.HEADING_STDDEV_IGNORE;
import static frc.robot.util.constants.VisionConstants.LINEAR_STDDEV_BASELINE;
import static frc.robot.util.constants.VisionConstants.MAX_ABS_TILT_DEGREES_FOR_VISION;
import static frc.robot.util.constants.VisionConstants.MAX_AMBIGUITY;
import static frc.robot.util.constants.VisionConstants.MAX_AVG_TAG_DISTANCE_METERS;
import static frc.robot.util.constants.VisionConstants.MAX_FRAME_AGE_SECONDS;
import static frc.robot.util.constants.VisionConstants.MAX_POSE_DELTA_METERS;
import static frc.robot.util.constants.VisionConstants.MAX_Z_ERROR;
import static frc.robot.util.constants.VisionConstants.MIN_TRANSLATION_STDDEV_METERS;
import static frc.robot.util.constants.VisionConstants.MULTITAG_INIT_MAX_HEADING_DELTA_DEGREES;
import static frc.robot.util.constants.VisionConstants.MULTITAG_INIT_MAX_TRANSLATION_DELTA_METERS;
import static frc.robot.util.constants.VisionConstants.MULTITAG_INIT_STABLE_POSES_REQUIRED;
import static frc.robot.util.constants.VisionConstants.SINGLE_TAG_LINEAR_STDDEV_MULTIPLIER;
import static frc.robot.util.constants.VisionConstants.SNAPSHOT_MAX_AGE_SECONDS;

import com.ctre.phoenix6.Utils;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiConsumer;

/**
 * Lean AprilTag pose-estimation subsystem.
 *
 * <p>Design: {@code docs/superpowers/specs/2026-09-26-vision-ctre-fusion-design.md}.
 *
 * <ul>
 *   <li>Vision NEVER hard-resets the drivetrain pose during normal operation. It only feeds
 *       weighted measurements through {@link VisionConsumer}; the pose estimator blends them.
 *   <li>Vision heading is ignored (huge angular std-dev); the gyro is authoritative.
 *   <li>Every accepted camera observation reaches CTRE with distance-scaled uncertainty.
 *   <li>Simple, readable rejection: tag count, Z, field bounds, single-tag ambiguity, max distance.
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {

  private final CommandSwerveDrivetrain swerve;
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;
  private final List<Pose3d> acceptedPoses = new ArrayList<>(16);
  private final List<Pose3d> rejectedPoses = new ArrayList<>(16);
  private final List<CameraObservation> pendingObservations = new ArrayList<>(16);
  private final RawObservationLogBuffers[] rawObservationLogBuffers;
  private final double[] lastProcessedTimestamps;
  private final int[] cameraConfigIndexes;
  private final BiConsumer<String, String> diagnosticWriter;
  private long observationSequence;
  private long frameSequence;

  /** Operator intent is logged; it does not change camera measurement quality. */
  private boolean aiming = false;

  /** Most recent accepted observation across all cameras (for the dashboard overlay). */
  private Pose2d lastAcceptedPose = null;

  private int[] lastAcceptedTagIDs = new int[0];
  private double lastAcceptedTimestamp = -1.0;
  private final Map<String, MultitagInitializationState> multitagInitializationByCamera =
      new HashMap<>();
  private int stableMultitagPoseCount = 0;
  private boolean visionInitializationComplete = false;
  private boolean hasAutoReseededThisDisabledCycle = false;
  private boolean hasEnteredEnabledModeSinceStartup = false;
  private double lastDisabledAutoReseedTime = Double.NEGATIVE_INFINITY;
  private String initializationCamera = "";
  private AcceptedObservationSnapshot initializationSnapshot;

  /**
   * Creates the vision subsystem.
   *
   * @param swerve drivetrain used for timestamped odometry samples and optional disabled reseed
   * @param consumer accepts filtered field-relative vision measurements in meters
   * @param io camera IO implementations, one per physical or simulated camera
   */
  public VisionSubsystem(CommandSwerveDrivetrain swerve, VisionConsumer consumer, VisionIO... io) {
    this(swerve, consumer, (key, value) -> DogLog.log(key, value), io);
  }

  /** Allows the same complete diagnostic records to be consumed by offline verification. */
  VisionSubsystem(
      CommandSwerveDrivetrain swerve,
      VisionConsumer consumer,
      BiConsumer<String, String> diagnosticWriter,
      VisionIO... io) {
    this.swerve = swerve;
    this.consumer = consumer;
    this.io = io;
    this.diagnosticWriter = diagnosticWriter;
    lastProcessedTimestamps = new double[io.length];
    cameraConfigIndexes = new int[io.length];
    Arrays.fill(lastProcessedTimestamps, Double.NEGATIVE_INFINITY);

    inputs = new VisionIOInputs[io.length];
    disconnectedAlerts = new Alert[io.length];
    rawObservationLogBuffers = new RawObservationLogBuffers[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputs();
      cameraConfigIndexes[i] = Arrays.asList(APTAG_CAMERA_NAMES).indexOf(io[i].getCameraName());
      DogLog.log(
          "Vision/" + io[i].getCameraName() + "/Configuration/StdDevFactor",
          cameraStdDevFactor(cameraConfigIndexes[i]));
      rawObservationLogBuffers[i] = new RawObservationLogBuffers();
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + io[i].getCameraName() + " is disconnected.", AlertType.kWarning);

      if (io[i] instanceof VisionIOPhotonVision photonVisionIo) {
        photonVisionIo.setHeadingProvider(new DrivetrainHeadingProvider());
      }
    }
    DogLog.log(
        "Vision/ActiveCameras",
        Arrays.stream(io).map(VisionIO::getCameraName).toArray(String[]::new));
  }

  @FunctionalInterface
  public interface VisionConsumer {
    /**
     * Feeds a filtered vision measurement to the drivetrain pose estimator.
     *
     * @param visionRobotPoseMeters field-relative robot pose in meters
     * @param timestampSeconds capture timestamp converted to CTRE's current-time epoch
     * @param visionMeasurementStdDevs x, y, and heading standard deviations
     */
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /** Immutable snapshot of the latest accepted observation, consumed by the dashboard overlay. */
  public static record AcceptedObservationSnapshot(Pose2d pose, int[] tagIDs, double timestamp) {}

  private record CameraObservation(
      int cameraIndex, String cameraName, PoseObservation observation) {}

  /** Sets aiming state for diagnostics; uncertainty depends on observation quality. */
  public void setAiming(boolean aiming) {
    this.aiming = aiming;
  }

  @Override
  public void periodic() {
    DogLog.time("Perf/Vision");

    acceptedPoses.clear();
    rejectedPoses.clear();
    pendingObservations.clear();
    shouldAutoReseedForRobotState(DriverStation.isEnabled());

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      processCamera(cameraIndex);
    }

    pendingObservations.sort(
        Comparator.comparingDouble((CameraObservation item) -> item.observation().timestamp())
            .thenComparingInt(CameraObservation::cameraIndex));
    for (CameraObservation item : pendingObservations) {
      processObservation(item.cameraIndex(), item.cameraName(), item.observation());
    }
    logPeriodicSummary();
    maybeAutoReseedWhileDisabled();

    DogLog.timeEnd("Perf/Vision");
  }

  private void processCamera(int cameraIndex) {
    io[cameraIndex].updateInputs(inputs[cameraIndex]);
    disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].isConnected());

    String cameraName = inputs[cameraIndex].getCameraName();
    String cameraLogKey = "Vision/" + cameraName;
    DogLog.log(cameraLogKey + "/Connected", inputs[cameraIndex].isConnected());
    PoseObservation[] observations = inputs[cameraIndex].getPoseObservations();

    logRawObservations(cameraLogKey, observations, rawObservationLogBuffers[cameraIndex]);
    for (VisionIO.FrameDiagnostic frame : inputs[cameraIndex].getFrameDiagnostics()) {
      if (!"POSE_OBSERVATION".equals(frame.status())) {
        diagnosticWriter.accept(
            cameraLogKey + "/Frame",
            VisionDiagnostics.frame(++frameSequence, cameraName, frame, Timer.getFPGATimestamp()));
      }
    }

    for (PoseObservation observation : observations) {
      pendingObservations.add(new CameraObservation(cameraIndex, cameraName, observation));
    }
  }

  private void processObservation(int cameraIndex, String cameraName, PoseObservation observation) {
    double now = Timer.getFPGATimestamp();
    double age = now - observation.timestamp();
    double ctreTimestamp = Utils.fpgaToCurrentTime(observation.timestamp());
    var before = swerve.getStateCopy();
    Pose2d reference =
        Double.isFinite(observation.timestamp())
            ? swerve.samplePoseAt(observation.timestamp()).orElse(null)
            : null;
    double innovation =
        reference == null
            ? Double.NaN
            : observation
                .pose()
                .toPose2d()
                .getTranslation()
                .getDistance(reference.getTranslation());
    Matrix<N3, N1> sigma =
        standardDeviations(observation, cameraConfigIndexes[cameraIndex], aiming);
    boolean startup =
        DriverStation.isDisabled()
            && !hasEnteredEnabledModeSinceStartup
            && !visionInitializationComplete;
    String reason = rejectionReason(observation).orElse("");
    if (!Double.isFinite(observation.timestamp())) reason = "INVALID_TIMESTAMP";
    else if (age < 0.0) reason = "FUTURE_TIMESTAMP";
    else if (age > MAX_FRAME_AGE_SECONDS) reason = "STALE_FRAME";
    else if (observation.timestamp() <= lastProcessedTimestamps[cameraIndex])
      reason = "DUPLICATE_OR_OUT_OF_ORDER";
    else {
      lastProcessedTimestamps[cameraIndex] = observation.timestamp();
      if (reason.isEmpty() && reference == null) reason = "NO_ODOMETRY_HISTORY";
      if (reason.isEmpty() && !swerve.isPitchRollStableForVision(MAX_ABS_TILT_DEGREES_FOR_VISION))
        reason = "TILT_UNSTABLE";
      if (reason.isEmpty() && innovation > MAX_POSE_DELTA_METERS && !startup) reason = "POSE_DELTA";
    }
    boolean accepted = reason.isEmpty();
    if (accepted) {
      Pose2d pose = observation.pose().toPose2d();
      consumer.accept(pose, ctreTimestamp, sigma);
      acceptedPoses.add(observation.pose());
      trackMultitagInitialization(observation, pose, cameraName);
      updateLatestAcceptedSnapshot(observation, pose, cameraName);
      if (hasAutoReseededThisDisabledCycle
          && cameraName.equals(initializationCamera)
          && isMultitagInitCandidate(observation)) {
        initializationSnapshot =
            new AcceptedObservationSnapshot(
                pose,
                Arrays.copyOf(observation.tagIDs(), observation.tagIDs().length),
                observation.timestamp());
      }
    } else {
      rejectObservation("Vision/" + cameraName, observation, reason);
    }
    int cameraCount =
        multitagInitializationByCamera.containsKey(cameraName)
            ? multitagInitializationByCamera.get(cameraName).stablePoseCount
            : 0;
    diagnosticWriter.accept(
        "Vision/" + cameraName + "/Observation",
        new VisionDiagnostics.Observation(
                ++observationSequence,
                cameraName,
                observation,
                now,
                ctreTimestamp,
                reference,
                innovation,
                sigma,
                accepted,
                reason,
                startup,
                before.Pose,
                swerve.getState().Pose,
                before.Speeds,
                swerve.getPitchDegrees(),
                swerve.getRollDegrees(),
                DriverStation.isEnabled(),
                DriverStation.isAutonomous(),
                cameraCount,
                initializationCamera,
                aiming)
            .toJson());
  }

  private void rejectObservation(
      String cameraLogKey, PoseObservation observation, String rejectedReason) {
    rejectedPoses.add(observation.pose());
    DogLog.log(cameraLogKey + "/RejectedReason", rejectedReason);
  }

  private void updateLatestAcceptedSnapshot(
      PoseObservation observation, Pose2d visionPose, String cameraName) {
    boolean disabled = DriverStation.isDisabled();
    boolean allowSnapshotUpdate =
        !disabled || observation.type() == PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR;
    if (observation.timestamp() > lastAcceptedTimestamp && allowSnapshotUpdate) {
      lastAcceptedPose = visionPose;
      lastAcceptedTagIDs = Arrays.copyOf(observation.tagIDs(), observation.tagIDs().length);
      lastAcceptedTimestamp = observation.timestamp();
    } else if (observation.timestamp() > lastAcceptedTimestamp) {
      DogLog.log(
          "Vision/DisabledAutoReseed/SnapshotRejected",
          cameraName + " type=" + observation.type().name());
    }
  }

  private void logPeriodicSummary() {
    logPoseArray("Vision/AcceptedPoses", acceptedPoses);
    logPoseArray("Vision/RejectedPoses", rejectedPoses);
    DogLog.log("Vision/Aiming", aiming);

    var drivetrainState = swerve.getState();
    double linearSpeedMetersPerSecond =
        Math.hypot(
            drivetrainState.Speeds.vxMetersPerSecond, drivetrainState.Speeds.vyMetersPerSecond);
    double angularSpeedRadiansPerSecond = drivetrainState.Speeds.omegaRadiansPerSecond;
    DogLog.log("Vision/RobotLinearSpeedMetersPerSecond", linearSpeedMetersPerSecond);
    DogLog.log("Vision/RobotAngularSpeedRadiansPerSecond", angularSpeedRadiansPerSecond);
    DogLog.log(
        "Vision/RobotAngularSpeedDegreesPerSecond", Math.toDegrees(angularSpeedRadiansPerSecond));
    DogLog.log("Vision/Initialization/StableMultitagPoseCount", stableMultitagPoseCount);
    DogLog.log("Vision/Initialization/Complete", visionInitializationComplete);
  }

  /**
   * While disabled, initialize (or refresh) odometry from the latest accepted vision pose after one
   * camera has supplied five stable accepted coprocessor MultiTag poses.
   */
  private void maybeAutoReseedWhileDisabled() {
    boolean autoReseedAllowed = shouldAutoReseedForRobotState(DriverStation.isEnabled());
    boolean disabled = DriverStation.isDisabled();
    DogLog.log("Vision/DisabledAutoReseed/Allowed", autoReseedAllowed);
    DogLog.log(
        "Vision/DisabledAutoReseed/HasEnteredEnabledMode", hasEnteredEnabledModeSinceStartup);
    if (!disabled) {
      DogLog.log("Vision/DisabledAutoReseed/SuppressedReason", "ROBOT_ENABLED");
      hasAutoReseededThisDisabledCycle = false;
      return;
    }
    if (!autoReseedAllowed) {
      DogLog.log("Vision/DisabledAutoReseed/SuppressedReason", "ENABLED_MODE_ALREADY_ENTERED");
      return;
    }
    if (stableMultitagPoseCount < MULTITAG_INIT_STABLE_POSES_REQUIRED) {
      DogLog.log("Vision/DisabledAutoReseed/SuppressedReason", "WAITING_FOR_STABLE_MULTITAG");
      return;
    }
    DogLog.log("Vision/DisabledAutoReseed/SuppressedReason", "");

    Optional<AcceptedObservationSnapshot> snapshot =
        Optional.ofNullable(initializationSnapshot)
            .filter(
                value -> Timer.getFPGATimestamp() - value.timestamp() <= SNAPSHOT_MAX_AGE_SECONDS);
    if (snapshot.isEmpty()) {
      if (!hasAutoReseededThisDisabledCycle && initializationSnapshot != null) {
        // Qualification expired before the initial reset. Require a new five-pose streak rather
        // than letting a later unqualified frame replace it or permanently blocking startup.
        initializationSnapshot = null;
        initializationCamera = "";
        multitagInitializationByCamera.clear();
        stableMultitagPoseCount = 0;
        visionInitializationComplete = false;
        for (VisionIO visionIo : io) {
          if (visionIo instanceof VisionIOPhotonVision photonVisionIo) {
            photonVisionIo.restartVisionInitialization();
          }
        }
        DogLog.log("Vision/DisabledAutoReseed/SuppressedReason", "STARTUP_SNAPSHOT_EXPIRED");
      }
      return;
    }

    int tagCount = snapshot.get().tagIDs().length;
    if (tagCount < DISABLED_AUTO_RESEED_MIN_TAG_COUNT) {
      DogLog.log("Vision/DisabledAutoReseed/RejectedReason", "NEEDS_MULTITAG tagCount=" + tagCount);
      return;
    }

    Pose2d currentPose = swerve.getState().Pose;
    Pose2d visionPose = snapshot.get().pose();
    double poseDeltaMeters = currentPose.getTranslation().getDistance(visionPose.getTranslation());

    double now = Timer.getFPGATimestamp();
    boolean needsInitialReseed = !hasAutoReseededThisDisabledCycle;
    boolean intervalElapsed =
        (now - lastDisabledAutoReseedTime) >= DISABLED_AUTO_RESEED_MIN_INTERVAL_SECONDS;
    boolean drifted = poseDeltaMeters > DISABLED_AUTO_RESEED_DELTA_METERS;

    if ((needsInitialReseed || drifted) && intervalElapsed) {
      swerve.resetPose(visionPose, "VISION_STARTUP:" + initializationCamera);
      hasAutoReseededThisDisabledCycle = true;
      lastDisabledAutoReseedTime = now;
      DogLog.log("Vision/DisabledAutoReseed/Pose", visionPose);
      DogLog.log("Vision/DisabledAutoReseed/DeltaMeters", poseDeltaMeters);
      DogLog.log("Vision/DisabledAutoReseed/Timestamp", snapshot.get().timestamp());
    }
  }

  /**
   * Records whether this program run has ever seen the robot enabled and reports whether an
   * automatic disabled vision reseed is currently permitted. A Driver Station disconnect cannot
   * reopen reseeding, while a program restart creates a fresh subsystem and permits startup
   * localization again.
   */
  boolean shouldAutoReseedForRobotState(boolean robotEnabled) {
    if (robotEnabled) {
      hasEnteredEnabledModeSinceStartup = true;
    }
    return !robotEnabled && !hasEnteredEnabledModeSinceStartup;
  }

  private void markVisionInitializationComplete() {
    if (visionInitializationComplete) {
      return;
    }
    visionInitializationComplete = true;
    for (VisionIO visionIo : io) {
      if (visionIo instanceof VisionIOPhotonVision photonVisionIo) {
        photonVisionIo.markVisionInitializationComplete();
      }
    }
  }

  private void trackMultitagInitialization(
      PoseObservation observation, Pose2d pose2d, String cameraName) {
    if (visionInitializationComplete) {
      return;
    }

    boolean isMultitagCoprocessor = isMultitagInitCandidate(observation);
    if (!isMultitagCoprocessor) {
      return;
    }
    MultitagInitializationState initState =
        multitagInitializationByCamera.computeIfAbsent(
            cameraName, unused -> new MultitagInitializationState());

    double translationDelta = 0.0;
    double headingDeltaDeg = 0.0;
    boolean isStable = true;
    if (initState.lastStablePose != null) {
      translationDelta =
          pose2d.getTranslation().getDistance(initState.lastStablePose.getTranslation());
      headingDeltaDeg =
          Math.abs(pose2d.getRotation().minus(initState.lastStablePose.getRotation()).getDegrees());
      isStable =
          isStableMultitagStep(
              observation.timestamp(),
              initState.lastStableTimestamp,
              translationDelta,
              headingDeltaDeg);
    }

    initState.stablePoseCount = nextStableMultitagPoseCount(initState.stablePoseCount, isStable);
    initState.lastStablePose = pose2d;
    initState.lastStableTimestamp = observation.timestamp();
    refreshStableMultitagPoseCount();

    DogLog.log("Vision/Initialization/Camera", cameraName);
    DogLog.log("Vision/Initialization/TranslationDeltaMeters", translationDelta);
    DogLog.log("Vision/Initialization/HeadingDeltaDegrees", headingDeltaDeg);

    if (initState.stablePoseCount >= MULTITAG_INIT_STABLE_POSES_REQUIRED) {
      initializationCamera = cameraName;
      initializationSnapshot =
          new AcceptedObservationSnapshot(
              pose2d,
              Arrays.copyOf(observation.tagIDs(), observation.tagIDs().length),
              observation.timestamp());
      markVisionInitializationComplete();
      DogLog.log("Vision/Initialization/StableMultitagPoseTimestamp", observation.timestamp());
    }
  }

  private void refreshStableMultitagPoseCount() {
    int maxStablePoseCount = 0;
    for (MultitagInitializationState initState : multitagInitializationByCamera.values()) {
      maxStablePoseCount = Math.max(maxStablePoseCount, initState.stablePoseCount);
    }
    stableMultitagPoseCount = maxStablePoseCount;
  }

  static boolean isMultitagInitCandidate(PoseObservation observation) {
    return observation.type() == PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR
        && observation.tagCount() >= DISABLED_AUTO_RESEED_MIN_TAG_COUNT;
  }

  static boolean isStableMultitagStep(
      double timestamp,
      double previousTimestamp,
      double translationDeltaMeters,
      double headingDeltaDegrees) {
    return timestamp > previousTimestamp
        && translationDeltaMeters <= MULTITAG_INIT_MAX_TRANSLATION_DELTA_METERS
        && headingDeltaDegrees <= MULTITAG_INIT_MAX_HEADING_DELTA_DEGREES;
  }

  static int nextStableMultitagPoseCount(int currentCount, boolean isStableStep) {
    return isStableStep ? (currentCount + 1) : 1;
  }

  static int requiredStableMultitagPosesForInitialization() {
    return MULTITAG_INIT_STABLE_POSES_REQUIRED;
  }

  private static class MultitagInitializationState {
    private Pose2d lastStablePose = null;
    private double lastStableTimestamp = Double.NEGATIVE_INFINITY;
    private int stablePoseCount = 0;
  }

  /**
   * Returns a human-readable rejection reason, or empty if the observation should be accepted.
   *
   * <p>Reject when: no tags, unrealistic Z, outside the field, a single tag with high ambiguity, or
   * the average tag distance exceeds {@link
   * frc.robot.util.constants.VisionConstants#MAX_AVG_TAG_DISTANCE_METERS}.
   *
   * <p>Static and package-private so tests can exercise the real gate logic without a HAL/sim
   * drivetrain.
   */
  static Optional<String> rejectionReason(PoseObservation observation) {
    if (observation.tagCount() <= 0) {
      return Optional.of("NO_TAGS");
    }

    Pose3d pose = observation.pose();
    if (!Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getZ())
        || !Double.isFinite(pose.getRotation().getX())
        || !Double.isFinite(pose.getRotation().getY())
        || !Double.isFinite(pose.getRotation().getZ())
        || !Double.isFinite(observation.averageTagDistance())
        || observation.averageTagDistance() <= 0.0
        || !Double.isFinite(observation.ambiguity())) {
      return Optional.of("NONFINITE_OR_INVALID_MEASUREMENT");
    }
    if (observation.tagIDs().length != observation.tagCount()
        || Arrays.stream(observation.tagIDs()).distinct().count() != observation.tagCount()
        || Arrays.stream(observation.tagIDs())
            .anyMatch(id -> APTAG_FIELD_LAYOUT.getTagPose(id).isEmpty())) {
      return Optional.of("INVALID_TAG_IDS");
    }
    if (Math.abs(pose.getZ()) > MAX_Z_ERROR) {
      return Optional.of("Z=" + pose.getZ());
    }

    Pose2d pose2d = pose.toPose2d();
    if (pose2d.getX() < 0.0
        || pose2d.getX() > APTAG_FIELD_LAYOUT.getFieldLength()
        || pose2d.getY() < 0.0
        || pose2d.getY() > APTAG_FIELD_LAYOUT.getFieldWidth()) {
      return Optional.of("OUT_OF_BOUNDS");
    }

    if (observation.tagCount() == 1
        && (observation.ambiguity() < 0.0 || observation.ambiguity() > MAX_AMBIGUITY)) {
      return Optional.of("AMBIGUITY=" + observation.ambiguity());
    }

    if (observation.averageTagDistance() > MAX_AVG_TAG_DISTANCE_METERS) {
      return Optional.of("DISTANCE=" + observation.averageTagDistance());
    }

    return Optional.empty();
  }

  /**
   * Distance-scaled uncertainty with heading ignored. The aiming argument is retained for
   * comparison callers; operator intent does not change measurement trust.
   */
  static Matrix<N3, N1> standardDeviations(
      PoseObservation observation, int cameraIndex, boolean aiming) {
    double tagCount = Math.max(observation.tagCount(), 1);
    double rawDistance = observation.averageTagDistance();
    // Guard against missing/invalid distance samples from the IO layer.
    double distance =
        Double.isFinite(rawDistance) && rawDistance > 0.0
            ? rawDistance
            : MAX_AVG_TAG_DISTANCE_METERS;
    double factor = (distance * distance) / tagCount;

    double cameraFactor = cameraStdDevFactor(cameraIndex);
    double singleTagFactor =
        (observation.tagCount() == 1) ? SINGLE_TAG_LINEAR_STDDEV_MULTIPLIER : 1.0;

    double linearStdDev = LINEAR_STDDEV_BASELINE * factor * cameraFactor * singleTagFactor;
    linearStdDev = Math.max(linearStdDev, MIN_TRANSLATION_STDDEV_METERS);

    return VecBuilder.fill(linearStdDev, linearStdDev, HEADING_STDDEV_IGNORE);
  }

  private static double cameraStdDevFactor(int configIndex) {
    return configIndex >= 0 && configIndex < CAMERA_STDDEV_FACTORS.length
        ? CAMERA_STDDEV_FACTORS[configIndex]
        : 1.0;
  }

  /** Logs a pose list as a struct array for AdvantageScope. */
  @SuppressWarnings("null") // DogLog null-annotation interop on Pose3d[] is benign.
  private static void logPoseArray(String key, List<Pose3d> poses) {
    DogLog.log(key, poses.toArray(new Pose3d[0]));
  }

  /**
   * Logs pre-filter pose arrays for field plots. Atomic Observation records additionally preserve
   * solver metadata, decisions, uncertainty and drivetrain context for offline diagnosis.
   */
  @SuppressWarnings("null") // DogLog null-annotation interop on Pose3d[] is benign.
  private static void logRawObservations(
      String camKey, PoseObservation[] observations, RawObservationLogBuffers buffers) {
    int count = observations.length;
    buffers.ensureCapacity(count);

    Pose3d[] poses = buffers.poses;
    double[] timestamps = buffers.timestamps;
    double[] ambiguities = buffers.ambiguities;
    double[] tagCounts = buffers.tagCounts;
    double[] avgDistances = buffers.avgDistances;

    for (int i = 0; i < observations.length; i++) {
      poses[i] = observations[i].pose();
      timestamps[i] = observations[i].timestamp();
      ambiguities[i] = observations[i].ambiguity();
      tagCounts[i] = observations[i].tagCount();
      avgDistances[i] = observations[i].averageTagDistance();
    }
    DogLog.log(camKey + "/RawObs/Count", count);
    DogLog.log(camKey + "/RawObs/Poses", Arrays.copyOf(poses, count));
    DogLog.log(camKey + "/RawObs/Timestamp", Arrays.copyOf(timestamps, count));
    DogLog.log(camKey + "/RawObs/Ambiguity", Arrays.copyOf(ambiguities, count));
    DogLog.log(camKey + "/RawObs/TagCount", Arrays.copyOf(tagCounts, count));
    DogLog.log(camKey + "/RawObs/AvgDistance", Arrays.copyOf(avgDistances, count));
  }

  private static class RawObservationLogBuffers {
    private Pose3d[] poses = new Pose3d[0];
    private double[] timestamps = new double[0];
    private double[] ambiguities = new double[0];
    private double[] tagCounts = new double[0];
    private double[] avgDistances = new double[0];

    private void ensureCapacity(int count) {
      if (poses.length >= count) {
        return;
      }

      int newCapacity = Math.max(count, poses.length * 2 + 1);
      poses = Arrays.copyOf(poses, newCapacity);
      timestamps = Arrays.copyOf(timestamps, newCapacity);
      ambiguities = Arrays.copyOf(ambiguities, newCapacity);
      tagCounts = Arrays.copyOf(tagCounts, newCapacity);
      avgDistances = Arrays.copyOf(avgDistances, newCapacity);
    }
  }

  /** Returns a consistent snapshot of the latest accepted observation, if recent. */
  public Optional<AcceptedObservationSnapshot> getLatestAcceptedObservationSnapshot() {
    if (lastAcceptedPose == null
        || (Timer.getFPGATimestamp() - lastAcceptedTimestamp) > SNAPSHOT_MAX_AGE_SECONDS) {
      return Optional.empty();
    }
    return Optional.of(
        new AcceptedObservationSnapshot(
            lastAcceptedPose,
            Arrays.copyOf(lastAcceptedTagIDs, lastAcceptedTagIDs.length),
            lastAcceptedTimestamp));
  }

  /**
   * Operator-triggered recovery: snaps the drivetrain pose to the most recent accepted vision pose.
   * Note: the subsystem may also auto-reseed while disabled for pre-match localization.
   *
   * @return true if a recent accepted pose was available
   */
  public boolean forceReseedFromVision() {
    Optional<AcceptedObservationSnapshot> snapshot = getLatestAcceptedObservationSnapshot();
    if (snapshot.isEmpty()) {
      return false;
    }
    swerve.resetPose(snapshot.get().pose(), "OPERATOR_VISION");
    DogLog.log("Vision/ForceReseed", snapshot.get().pose());
    return true;
  }

  /** Feeds the drivetrain heading to PhotonVision for single-tag constrained solving. */
  private class DrivetrainHeadingProvider implements VisionIOPhotonVision.VisionHeadingProvider {
    @Override
    public Optional<Rotation2d> getHeadingAtTimestamp(double fpgaTimestampSeconds) {
      return swerve.samplePoseAt(fpgaTimestampSeconds).map(Pose2d::getRotation);
    }

    @Override
    public Optional<Pose3d> getSeedPoseAtTimestamp(double fpgaTimestampSeconds) {
      return swerve.samplePoseAt(fpgaTimestampSeconds).map(Pose3d::new);
    }

    @Override
    public double getAngularRateRadPerSec() {
      return swerve.getState().Speeds.omegaRadiansPerSecond;
    }

    @Override
    public double getLinearSpeedMetersPerSecond() {
      double vx = swerve.getState().Speeds.vxMetersPerSecond;
      double vy = swerve.getState().Speeds.vyMetersPerSecond;
      return Math.hypot(vx, vy);
    }
  }
}
