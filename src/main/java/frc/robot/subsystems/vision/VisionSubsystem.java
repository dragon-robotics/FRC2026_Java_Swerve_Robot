// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.util.constants.VisionConstants.*;

import com.ctre.phoenix6.Utils;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.vision.AcceptedPose;
import frc.robot.util.vision.RejectedPose;
import frc.robot.util.vision.VisionPoseValidator;
import java.util.Arrays;
import java.util.concurrent.locks.ReentrantLock;

public class VisionSubsystem extends SubsystemBase {

  private final CommandSwerveDrivetrain swerve;
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final Alert[] disconnectedAlerts;

  // -- Threading: double-buffered inputs --
  // The background thread writes into 'threadInputs', then copies to
  // 'latestInputs' under lock.
  // The main thread copies 'latestInputs' into 'mainInputs' under lock, then
  // processes freely.
  private final VisionIOInputs[] threadInputs; // owned by background thread only
  private final VisionIOInputs[] latestInputs; // shared, protected by lock
  private final VisionIOInputs[] mainInputs; // owned by main thread only
  private final ReentrantLock inputsLock = new ReentrantLock();
  private final Thread visionThread;

  // Check for odometry initialization
  private int stablePoseCounter = 5;
  private boolean odometryInitialized = false;

  // Vision pose validator
  private final VisionPoseValidator poseValidator = new VisionPoseValidator();

  // Tracks the name of the camera being processed, so handleAcceptedPose can log
  // per-camera keys
  private String currentCameraName = "";

  // Pre-allocated pose buffer -- avoids ArrayList and toArray() allocation each
  // cycle.
  private Pose3d[] acceptedPoseBuffer;
  private int acceptedPoseCount = 0;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(CommandSwerveDrivetrain swerve, VisionConsumer consumer, VisionIO... io) {
    this.swerve = swerve;
    this.consumer = consumer;
    this.io = io;

    // Initialize triple-buffered inputs and alerts
    threadInputs = new VisionIOInputs[io.length];
    latestInputs = new VisionIOInputs[io.length];
    mainInputs = new VisionIOInputs[io.length];
    disconnectedAlerts = new Alert[io.length];

    for (int i = 0; i < io.length; i++) {
      threadInputs[i] = new VisionIOInputs();
      latestInputs[i] = new VisionIOInputs();
      mainInputs[i] = new VisionIOInputs();
      disconnectedAlerts[i] = new Alert(
          "Vision camera " + io[i].getCameraName() + " is disconnected.", AlertType.kWarning);
    }

    // Pre-allocate pose buffer: each camera can produce ~2 poses max per cycle
    acceptedPoseBuffer = new Pose3d[io.length * 2];

    // Start background vision thread -- iterates all cameras in a loop
    visionThread = new Thread(this::visionThreadLoop, "VisionThread");
    visionThread.setDaemon(true);
    visionThread.start();
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /**
   * Background thread loop: continuously polls all cameras and publishes results.
   * Runs as fast as
   * cameras produce frames (~30fps). The thread owns 'threadInputs' exclusively
   * and only touches
   * 'latestInputs' under the lock for a fast memcpy.
   */
  private void visionThreadLoop() {
    while (!Thread.currentThread().isInterrupted()) {
      try {
        // Poll every camera sequentially on this thread
        for (int i = 0; i < io.length; i++) {
          io[i].updateInputs(threadInputs[i]);
        }

        // Publish results to shared buffer under lock (fast -- just field copies)
        inputsLock.lock();
        try {
          for (int i = 0; i < io.length; i++) {
            latestInputs[i].copyFrom(threadInputs[i]);
          }
        } finally {
          inputsLock.unlock();
        }

        // Yield briefly to avoid busy-spinning if cameras have no new data
        Thread.sleep(2);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        break;
      } catch (Exception e) {
        // Log but don't crash the thread on transient errors
        DogLog.log("Vision/ThreadError", e.getMessage());
      }
    }
  }

  @Override
  public void periodic() {
    // Snapshot latest camera results from background thread (fast lock)
    inputsLock.lock();
    try {
      for (int i = 0; i < io.length; i++) {
        mainInputs[i].copyFrom(latestInputs[i]);
      }
    } finally {
      inputsLock.unlock();
    }

    // Reset pose count -- no clearing needed, just overwrite slots
    acceptedPoseCount = 0;

    // Process all camera data on the main thread (validation + pose estimator
    // updates)
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      currentCameraName = mainInputs[cameraIndex].getCameraName();
      processCameraData(cameraIndex, mainInputs[cameraIndex]);
    }
  }

  private void processCameraData(int cameraIndex, VisionIOInputs inputs) {
    // Update disconnected alert
    disconnectedAlerts[cameraIndex].set(!inputs.isConnected());

    // Process pose observations -- no intermediate collections needed
    for (var observation : inputs.getPoseObservations()) {

      // Use instanceof with pattern matching
      var validationResult = poseValidator.validatePose(observation);

      if (validationResult instanceof AcceptedPose accepted) {
        handleAcceptedPose(accepted);
      } else if (validationResult instanceof RejectedPose rejected) {
        handleRejectedPose(rejected);
      }
    }
  }

  private void handleAcceptedPose(AcceptedPose accepted) {
    // Cache the poseObservation record accessor to avoid repeated calls
    var poseObs = accepted.poseObservation();
    Pose2d visionPose = poseObs.pose().toPose2d();
    String camKey = "Vision/" + currentCameraName;

    // Reject if vision pose disagrees with odometry by more than threshold.
    // Prevents a misidentified tag from snapping the pose estimator across the
    // field.
    // Skip this check until odometry is initialized — the robot starts at (0,0,0)
    // by default, so every real field pose would exceed MAX_POSE_DISCREPANCY_METERS
    // and prevent initialization from ever happening.
    if (odometryInitialized) {
      double poseDiscrepancy = swerve.getState().Pose.getTranslation().getDistance(visionPose.getTranslation());
      if (poseDiscrepancy > MAX_POSE_DISCREPANCY_METERS) {
        return;
      }
    }

    // Write directly into pre-allocated buffer -- grow if needed (rare)
    if (acceptedPoseCount >= acceptedPoseBuffer.length) {
      acceptedPoseBuffer = Arrays.copyOf(acceptedPoseBuffer, acceptedPoseBuffer.length * 2);
    }
    acceptedPoseBuffer[acceptedPoseCount++] = poseObs.pose();

    // Check if odometry is initialized
    if (!odometryInitialized) {
      stablePoseCounter--;
      if (stablePoseCounter <= 0) {
        swerve.resetPose(visionPose);
        odometryInitialized = true;
        DogLog.log("Vision/OdometryInitialized", true);
      }
    }

    // Add to pose estimator
    var stdDevs = calculateStandardDeviations(accepted);
    consumer.accept(visionPose, Utils.fpgaToCurrentTime(poseObs.timestamp()), stdDevs);

    // ── Logging ────────────────────────────────────────────────────────────
    DogLog.log(camKey + "/AcceptedVisionPose", visionPose);
  }

  private void handleRejectedPose(RejectedPose rejected) {
    // If odometry not initialized, reset stable pose counter
    if (!odometryInitialized) {
      stablePoseCounter = 5;
    }
  }

  private Matrix<N3, N1> calculateStandardDeviations(AcceptedPose accepted) {
    var poseObs = accepted.poseObservation();
    double dist = poseObs.averageTagDistance();
    int tagCount = Math.max(poseObs.tagCount(), 1);

    // Quadratic distance scaling: trust drops sharply with range.
    // Divide by tagCount so multiple tags reduce uncertainty.
    // (1 + ambiguity) penalises ambiguous single-tag solutions.
    double distanceFactor = (dist * dist) / tagCount;
    double linearStdDev = LINEAR_STDDEV_BASELINE * (1.0 + distanceFactor) * (1.0 + poseObs.ambiguity());

    // Never trust rotation from a single tag — ambiguity means we can't know which
    // of the two possible solutions is correct.
    double angularStdDev = (poseObs.tagCount() >= 2)
        ? ANGULAR_STDDEV_BASELINE * (1.0 + distanceFactor)
        : Double.MAX_VALUE;

    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }
}
