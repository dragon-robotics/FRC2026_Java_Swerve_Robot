#pragma once

#include <functional>
#include <optional>

#include <frc/geometry/Rotation2d.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>

class CommandSwerveDrivetrain;

class DefaultDriveCmd
    : public frc2::CommandHelper<frc2::Command, DefaultDriveCmd> {
 public:
  DefaultDriveCmd(
      CommandSwerveDrivetrain* swerve,
      std::function<double()> translationSup,
      std::function<double()> strafeSup,
      std::function<double()> rotationSup,
      std::function<bool()> halfSpeedSup,
      std::function<std::optional<frc::Rotation2d>()> headingGetter,
      std::function<void(std::optional<frc::Rotation2d>)> headingSetter,
      std::function<double()> rotationLastTriggeredGetter,
      std::function<void(double)> rotationLastTriggeredSetter);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  struct Speeds {
    double translation;
    double strafe;
    double rotation;
  };

  Speeds ProcessJoystickInputs(double rawTranslation, double rawStrafe,
                               double rawRotation, bool halfSpeed);
  void SetSwerveToRotate(double translation, double strafe, double rotation);
  void SetSwerveToMaintainHeading(double translation, double strafe);

  CommandSwerveDrivetrain* m_swerve;
  std::function<double()> m_translationSup;
  std::function<double()> m_strafeSup;
  std::function<double()> m_rotationSup;
  std::function<bool()> m_halfSpeedSup;
  std::function<std::optional<frc::Rotation2d>()> m_headingGetter;
  std::function<void(std::optional<frc::Rotation2d>)> m_headingSetter;
  std::function<double()> m_rotationLastTriggeredGetter;
  std::function<void(double)> m_rotationLastTriggeredSetter;

  double m_maxSpeed;
  double m_maxAngularRate;

  ctre::phoenix6::swerve::requests::FieldCentric m_drive;
  ctre::phoenix6::swerve::requests::FieldCentricFacingAngle m_driveMaintainHeading;
};
