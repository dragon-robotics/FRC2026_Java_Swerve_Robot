#pragma once

#include <functional>
#include <optional>

#include <frc/geometry/Rotation2d.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>

class CommandSwerveDrivetrain;

class ShootDriveCmd
    : public frc2::CommandHelper<frc2::Command, ShootDriveCmd> {
 public:
  ShootDriveCmd(
      CommandSwerveDrivetrain* swerve,
      std::function<double()> translationSup,
      std::function<double()> strafeSup,
      std::function<void(std::optional<frc::Rotation2d>)> setCurrentHeading);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  CommandSwerveDrivetrain* m_swerve;
  std::function<double()> m_translationSup;
  std::function<double()> m_strafeSup;
  std::function<void(std::optional<frc::Rotation2d>)> m_setCurrentHeading;

  double m_maxSpeed;
  double m_maxAngularRate;

  ctre::phoenix6::swerve::requests::FieldCentricFacingAngle m_driveMaintainHeading;
};
