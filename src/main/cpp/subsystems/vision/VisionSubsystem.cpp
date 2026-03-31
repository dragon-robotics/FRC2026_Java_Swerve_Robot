#include "subsystems/vision/VisionSubsystem.h"

#include <cmath>
#include <limits>

#include <ctre/phoenix6/Utils.hpp>
#include <frc/DataLogManager.h>

#include "Constants.h"
#include "subsystems/CommandSwerveDrivetrain.h"

using namespace vision;

VisionSubsystem::VisionSubsystem(
    CommandSwerveDrivetrain* swerve,
    VisionConsumer consumer,
    std::vector<std::unique_ptr<VisionIO>> io)
    : m_swerve{swerve},
      m_consumer{std::move(consumer)},
      m_io{std::move(io)} {
  // Initialize inputs and alerts
  m_threadInputs.resize(m_io.size());
  m_latestInputs.resize(m_io.size());
  m_mainInputs.resize(m_io.size());

  for (size_t i = 0; i < m_io.size(); i++) {
    m_disconnectedAlerts.emplace_back(
        "Vision camera " + m_io[i]->GetCameraName() + " is disconnected.",
        frc::Alert::AlertType::kWarning);
  }

  // Start background vision thread
  m_visionThread = std::thread{&VisionSubsystem::VisionThreadLoop, this};
}

VisionSubsystem::~VisionSubsystem() {
  m_threadRunning = false;
  if (m_visionThread.joinable()) {
    m_visionThread.join();
  }
}

void VisionSubsystem::VisionThreadLoop() {
  while (m_threadRunning) {
    // Poll every camera sequentially
    for (size_t i = 0; i < m_io.size(); i++) {
      m_io[i]->UpdateInputs(m_threadInputs[i]);
    }

    // Publish results to shared buffer under lock
    {
      std::lock_guard lock{m_inputsMutex};
      for (size_t i = 0; i < m_io.size(); i++) {
        m_latestInputs[i].CopyFrom(m_threadInputs[i]);
      }
    }

    // Yield briefly to avoid busy-spinning
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
}

void VisionSubsystem::Periodic() {
  // Snapshot latest camera results from background thread
  {
    std::lock_guard lock{m_inputsMutex};
    for (size_t i = 0; i < m_io.size(); i++) {
      m_mainInputs[i].CopyFrom(m_latestInputs[i]);
    }
  }

  // Process all camera data
  for (size_t cameraIndex = 0; cameraIndex < m_io.size(); cameraIndex++) {
    m_currentCameraName = m_mainInputs[cameraIndex].cameraName;
    ProcessCameraData(static_cast<int>(cameraIndex), m_mainInputs[cameraIndex]);
  }
}

void VisionSubsystem::ProcessCameraData(int cameraIndex, VisionIOInputs const& inputs) {
  m_disconnectedAlerts[cameraIndex].Set(!inputs.connected);

  for (auto const& observation : inputs.poseObservations) {
    auto result = m_poseValidator.ValidatePose(observation);

    if (result.accepted) {
      HandleAcceptedPose(observation, "Vision/" + m_currentCameraName);
    } else {
      HandleRejectedPose();
    }
  }
}

void VisionSubsystem::HandleAcceptedPose(PoseObservation const& poseObs,
                                          std::string const& camKey) {
  frc::Pose2d visionPose = poseObs.pose.ToPose2d();
  frc::Pose2d odometryPose = m_swerve->GetState().Pose;

  double poseDiscrepancy =
      odometryPose.Translation().Distance(visionPose.Translation()).value();

  if (poseDiscrepancy > VisionConstants::kMaxPoseDiscrepancyMeters) {
    if (!m_odometryInitialized) {
      m_stablePoseCounter = 5;
    }
    return;
  }

  if (!m_odometryInitialized) {
    m_stablePoseCounter--;
    if (m_stablePoseCounter <= 0) {
      m_swerve->ResetPose(visionPose);
      m_odometryInitialized = true;
    }
  }

  auto stdDevs = CalculateStandardDeviations(poseObs);
  m_consumer(visionPose,
             ctre::phoenix6::utils::FPGAToCurrentTime(units::second_t{poseObs.timestamp}),
             stdDevs);
}

void VisionSubsystem::HandleRejectedPose() {
  if (!m_odometryInitialized) {
    m_stablePoseCounter = 5;
  }
}

Eigen::Vector3d VisionSubsystem::CalculateStandardDeviations(
    PoseObservation const& poseObs) {
  double dist = poseObs.averageTagDistance;
  int tagCount = std::max(poseObs.tagCount, 1);

  double distanceFactor = (dist * dist) / tagCount;
  double linearStdDev = VisionConstants::kLinearStdDevBaseline *
                         (1.0 + distanceFactor) * (1.0 + poseObs.ambiguity);

  double angularStdDev = (poseObs.tagCount >= 2)
      ? VisionConstants::kAngularStdDevBaseline * (1.0 + distanceFactor)
      : std::numeric_limits<double>::max();

  return Eigen::Vector3d{linearStdDev, linearStdDev, angularStdDev};
}
