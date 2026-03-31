#pragma once

#include <functional>
#include <optional>

#include <frc/geometry/Rotation2d.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>

class CommandSwerveDrivetrain;

class AimAtTargetPoseCmd
    : public frc2::CommandHelper<frc2::Command, AimAtTargetPoseCmd> {
 public:
  AimAtTargetPoseCmd(
      CommandSwerveDrivetrain* swerve,
      std::function<void(std::optional<frc::Rotation2d>)> setCurrentHeading);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  CommandSwerveDrivetrain* m_swerve;
  std::function<void(std::optional<frc::Rotation2d>)> m_setCurrentHeading;

  double m_maxSpeed = 0.0;
  double m_maxAngularRate = 0.0;

  ctre::phoenix6::swerve::requests::FieldCentricFacingAngle m_driveMaintainHeading;
};
