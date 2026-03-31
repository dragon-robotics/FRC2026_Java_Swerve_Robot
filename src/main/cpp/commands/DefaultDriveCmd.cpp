#include "commands/DefaultDriveCmd.h"

#include <numbers>

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/Timer.h>

#include "Constants.h"
#include "generated/TunerConstants.h"
#include "subsystems/CommandSwerveDrivetrain.h"

using namespace ctre::phoenix6;

DefaultDriveCmd::DefaultDriveCmd(
    CommandSwerveDrivetrain* swerve,
    std::function<double()> translationSup,
    std::function<double()> strafeSup,
    std::function<double()> rotationSup,
    std::function<bool()> halfSpeedSup,
    std::function<std::optional<frc::Rotation2d>()> headingGetter,
    std::function<void(std::optional<frc::Rotation2d>)> headingSetter,
    std::function<double()> rotationLastTriggeredGetter,
    std::function<void(double)> rotationLastTriggeredSetter)
    : m_swerve{swerve},
      m_translationSup{std::move(translationSup)},
      m_strafeSup{std::move(strafeSup)},
      m_rotationSup{std::move(rotationSup)},
      m_halfSpeedSup{std::move(halfSpeedSup)},
      m_headingGetter{std::move(headingGetter)},
      m_headingSetter{std::move(headingSetter)},
      m_rotationLastTriggeredGetter{std::move(rotationLastTriggeredGetter)},
      m_rotationLastTriggeredSetter{std::move(rotationLastTriggeredSetter)},
      m_maxSpeed{TunerConstants::kSpeedAt12Volts.value()},
      m_maxAngularRate{2.0 * std::numbers::pi}  // 1 rotation/s in rad/s
{
  m_drive.Deadband = m_maxSpeed * 0.05;
  m_drive.RotationalDeadband = m_maxAngularRate * 0.05;
  m_drive.DriveRequestType =
      swerve::SwerveModule::DriveRequestType::OpenLoopVoltage;
  m_drive.DesaturateWheelSpeeds = true;

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

void DefaultDriveCmd::Initialize() {}

void DefaultDriveCmd::Execute() {
  double rawRotation = m_rotationSup();
  auto speeds = ProcessJoystickInputs(m_translationSup(), m_strafeSup(),
                                      rawRotation, m_halfSpeedSup());

  bool rotationTriggered =
      std::abs(rawRotation) > SwerveConstants::kSwerveDeadband;
  bool rotationActive =
      std::abs(m_rotationLastTriggeredGetter() - frc::Timer::GetFPGATimestamp().value()) < 0.1 &&
      std::abs(m_swerve->GetState().Speeds.omega.value()) > (10.0 * std::numbers::pi / 180.0);

  if (rotationTriggered) {
    m_rotationLastTriggeredSetter(frc::Timer::GetFPGATimestamp().value());
  }

  if (rotationTriggered || rotationActive) {
    SetSwerveToRotate(speeds.translation, speeds.strafe, speeds.rotation);
  } else {
    SetSwerveToMaintainHeading(speeds.translation, speeds.strafe);
  }
}

void DefaultDriveCmd::End(bool interrupted) {}

bool DefaultDriveCmd::IsFinished() {
  return false;
}

DefaultDriveCmd::Speeds DefaultDriveCmd::ProcessJoystickInputs(
    double rawTranslation, double rawStrafe, double rawRotation, bool halfSpeed) {
  double translation =
      frc::ApplyDeadband(rawTranslation, SwerveConstants::kSwerveDeadband);
  double strafe =
      frc::ApplyDeadband(rawStrafe, SwerveConstants::kSwerveDeadband);
  double rotation =
      frc::ApplyDeadband(rawRotation, SwerveConstants::kSwerveDeadband);

  translation = std::copysign(translation * translation, translation);
  strafe = std::copysign(strafe * strafe, strafe);
  rotation = std::copysign(rotation * rotation, rotation);

  if (halfSpeed) {
    translation *= 0.35;
    strafe *= 0.35;
    rotation *= 0.35;
  }

  return Speeds{translation * m_maxSpeed, strafe * m_maxSpeed,
                rotation * m_maxAngularRate};
}

void DefaultDriveCmd::SetSwerveToRotate(double translation, double strafe,
                                        double rotation) {
  m_swerve->SetControl(m_drive.WithVelocityX(units::meters_per_second_t{translation})
                           .WithVelocityY(units::meters_per_second_t{strafe})
                           .WithRotationalRate(units::radians_per_second_t{rotation}));
  m_headingSetter(std::nullopt);
}

void DefaultDriveCmd::SetSwerveToMaintainHeading(double translation,
                                                  double strafe) {
  if (!m_headingGetter().has_value()) {
    m_headingSetter(m_swerve->GetState().Pose.Rotation());
  }

  auto alliance = frc::DriverStation::GetAlliance();
  auto heading = m_headingGetter();
  if (alliance.has_value() && heading.has_value()) {
    frc::Rotation2d targetDirection =
        alliance.value() == frc::DriverStation::Alliance::kBlue
            ? heading.value()
            : heading.value().RotateBy(frc::Rotation2d{units::degree_t{180}});
    m_swerve->SetControl(
        m_driveMaintainHeading.WithVelocityX(units::meters_per_second_t{translation})
            .WithVelocityY(units::meters_per_second_t{strafe})
            .WithTargetDirection(targetDirection));
  }
}
