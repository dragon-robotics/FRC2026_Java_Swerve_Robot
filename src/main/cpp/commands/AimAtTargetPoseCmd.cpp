#include "commands/AimAtTargetPoseCmd.h"

#include <numbers>
#include <cmath>

#include <frc/DriverStation.h>

#include "Constants.h"
#include "subsystems/CommandSwerveDrivetrain.h"

using namespace ctre::phoenix6;

AimAtTargetPoseCmd::AimAtTargetPoseCmd(
    CommandSwerveDrivetrain* swerve,
    std::function<void(std::optional<frc::Rotation2d>)> setCurrentHeading)
    : m_swerve{swerve},
      m_setCurrentHeading{std::move(setCurrentHeading)} {
  m_driveMaintainHeading.Deadband = m_maxSpeed * 0.05;
  m_driveMaintainHeading.RotationalDeadband = m_maxAngularRate * 0.05;
  m_driveMaintainHeading.DriveRequestType =
      swerve::SwerveModule::DriveRequestType::OpenLoopVoltage;
  m_driveMaintainHeading.DesaturateWheelSpeeds = true;

  m_driveMaintainHeading.HeadingController.SetPID(
      SwerveConstants::kHeadingKP, SwerveConstants::kHeadingKI,
      SwerveConstants::kHeadingKD);
  m_driveMaintainHeading.HeadingController.EnableContinuousInput(-std::numbers::pi,
                                                                  std::numbers::pi);
  m_driveMaintainHeading.HeadingController.SetTolerance(
      SwerveConstants::kHeadingTolerance);

  AddRequirements(swerve);
}

void AimAtTargetPoseCmd::Initialize() {}

void AimAtTargetPoseCmd::Execute() {
  auto alliance = frc::DriverStation::GetAlliance();

  frc::Translation2d hubToAimTowards =
      (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed)
          ? FieldConstants::Hub::GetRedHubCenterPose()
          : FieldConstants::Hub::GetBlueHubCenterPose();

  auto robotTranslation = m_swerve->GetState().Pose.Translation();
  auto delta = hubToAimTowards - robotTranslation;

  frc::Rotation2d angleToPointAt{
      units::radian_t{std::atan2(delta.Y().value(), delta.X().value())}};

  m_setCurrentHeading(angleToPointAt);

  m_swerve->SetControl(m_driveMaintainHeading.WithTargetDirection(angleToPointAt)
                           .WithTargetRateFeedforward(units::radians_per_second_t{0.1}));
}

void AimAtTargetPoseCmd::End(bool interrupted) {}

bool AimAtTargetPoseCmd::IsFinished() {
  return false;
}
