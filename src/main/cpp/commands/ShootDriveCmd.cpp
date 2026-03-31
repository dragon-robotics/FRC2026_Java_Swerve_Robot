#include "commands/ShootDriveCmd.h"

#include <numbers>
#include <cmath>

#include <frc/DriverStation.h>

#include "Constants.h"
#include "generated/TunerConstants.h"
#include "subsystems/CommandSwerveDrivetrain.h"

using namespace ctre::phoenix6;

ShootDriveCmd::ShootDriveCmd(
    CommandSwerveDrivetrain* swerve,
    std::function<double()> translationSup,
    std::function<double()> strafeSup,
    std::function<void(std::optional<frc::Rotation2d>)> setCurrentHeading)
    : m_swerve{swerve},
      m_translationSup{std::move(translationSup)},
      m_strafeSup{std::move(strafeSup)},
      m_setCurrentHeading{std::move(setCurrentHeading)},
      m_maxSpeed{TunerConstants::kSpeedAt12Volts.value()},
      m_maxAngularRate{2.0 * std::numbers::pi} {
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

void ShootDriveCmd::Initialize() {}

void ShootDriveCmd::Execute() {
  double translation = (m_translationSup() * m_maxSpeed) / 3.0;
  double strafe = (m_strafeSup() * m_maxSpeed) / 3.0;

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

  m_swerve->SetControl(
      m_driveMaintainHeading.WithVelocityX(units::meters_per_second_t{translation})
          .WithVelocityY(units::meters_per_second_t{strafe})
          .WithTargetDirection(angleToPointAt)
          .WithTargetRateFeedforward(units::radians_per_second_t{0.1}));
}

void ShootDriveCmd::End(bool interrupted) {}

bool ShootDriveCmd::IsFinished() {
  return false;
}
