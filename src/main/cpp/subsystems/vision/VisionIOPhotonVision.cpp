#include "subsystems/vision/VisionIOPhotonVision.h"

#include <set>

#include "Constants.h"

using namespace vision;

VisionIOPhotonVision::VisionIOPhotonVision(
    std::string const& name,
    frc::Transform3d const& robotToCamera,
    std::function<SwerveDriveState()> swerveDriveStateSupplier)
    : m_camera{name},
      m_robotToCamera{robotToCamera},
      m_poseEstimator{FieldConstants::GetAprilTagFieldLayout(),
                      photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
                      robotToCamera},
      m_swerveDriveStateSupplier{std::move(swerveDriveStateSupplier)} {
  m_poseEstimator.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);
  m_poseEstimator.ResetHeadingData(
      frc::Timer::GetFPGATimestamp(),
      m_swerveDriveStateSupplier().Pose.Rotation());
}

void VisionIOPhotonVision::ResetHeadingData() {
  m_poseEstimator.ResetHeadingData(
      frc::Timer::GetFPGATimestamp(),
      m_swerveDriveStateSupplier().Pose.Rotation());
}

std::string VisionIOPhotonVision::GetCameraName() {
  return m_camera.GetCameraName();
}

void VisionIOPhotonVision::UpdateInputs(VisionIOInputs& inputs) {
  inputs.connected = m_camera.IsConnected();
  inputs.cameraName = m_camera.GetCameraName();

  m_poseEstimator.AddHeadingData(
      frc::Timer::GetFPGATimestamp(),
      m_swerveDriveStateSupplier().Pose.Rotation());

  std::set<int> tagIds;
  std::vector<PoseObservation> poseObservations;

  auto allResults = m_camera.GetAllUnreadResults();
  if (allResults.empty()) {
    inputs.poseObservations.clear();
    inputs.tagIds.clear();
    return;
  }

  auto& result = allResults.back();

  inputs.latestTargetObservation = TargetObservation{};

  if (result.HasTargets()) {
    auto bestTarget = result.GetBestTarget();

    inputs.latestTargetObservation = TargetObservation{
        frc::Rotation2d{units::degree_t{bestTarget.GetYaw()}},
        frc::Rotation2d{units::degree_t{bestTarget.GetPitch()}}};

    auto coprocResult = m_poseEstimator.EstimateCoprocMultiTagPose(result);
    std::optional<photon::EstimatedRobotPose> visionEst;
    if (coprocResult.has_value()) {
      visionEst = coprocResult;
    } else if (static_cast<int>(result.GetTargets().size()) >= 2) {
      visionEst = m_poseEstimator.Update(result);
    }

    if (visionEst.has_value()) {
      auto const& targets = result.GetTargets();
      int targetCount = static_cast<int>(targets.size());

      double totalDistance = 0.0;
      double totalAmbiguity = 0.0;

      for (auto const& target : targets) {
        totalDistance += target.GetBestCameraToTarget().Translation().Norm().value();
        totalAmbiguity += target.GetPoseAmbiguity();
        tagIds.insert(target.GetFiducialId());
      }

      poseObservations.push_back(PoseObservation{
          visionEst->estimatedPose.ToPose2d().X().value(),  // timestamp stored as seconds
          visionEst->estimatedPose,
          totalAmbiguity / targetCount,
          targetCount,
          totalDistance / targetCount,
          PoseObservationType::PHOTONVISION});
      // Fix: use the actual timestamp from the estimator
      poseObservations.back().timestamp = units::second_t{visionEst->timestamp}.value();
    }
  }

  inputs.poseObservations = std::move(poseObservations);
  inputs.tagIds.assign(tagIds.begin(), tagIds.end());
}
