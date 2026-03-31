#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc2/command/SubsystemBase.h>
#include <Eigen/Core>
#include <wpi/deprecated.h>
#include <frc/Alert.h>

#include "subsystems/vision/VisionIO.h"
#include "util/vision/VisionPoseValidator.h"

namespace vision {

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  using VisionConsumer = std::function<void(
      frc::Pose2d const&,
      double,
      Eigen::Vector3d const&)>;

  VisionSubsystem(
      class CommandSwerveDrivetrain* swerve,
      VisionConsumer consumer,
      std::vector<std::unique_ptr<VisionIO>> io);

  ~VisionSubsystem() override;

  void Periodic() override;

 private:
  void VisionThreadLoop();
  void ProcessCameraData(int cameraIndex, VisionIOInputs const& inputs);
  void HandleAcceptedPose(PoseObservation const& poseObs, std::string const& camKey);
  void HandleRejectedPose();
  Eigen::Vector3d CalculateStandardDeviations(PoseObservation const& poseObs);

  CommandSwerveDrivetrain* m_swerve;
  VisionConsumer m_consumer;
  std::vector<std::unique_ptr<VisionIO>> m_io;
  std::vector<frc::Alert> m_disconnectedAlerts;

  // Double-buffered inputs
  std::vector<VisionIOInputs> m_threadInputs;
  std::vector<VisionIOInputs> m_latestInputs;
  std::vector<VisionIOInputs> m_mainInputs;
  std::mutex m_inputsMutex;
  std::thread m_visionThread;
  bool m_threadRunning = true;

  // Odometry initialization
  int m_stablePoseCounter = 5;
  bool m_odometryInitialized = false;

  // Pose validation
  VisionPoseValidator m_poseValidator;

  // Current camera name for logging
  std::string m_currentCameraName;
};

}  // namespace vision
