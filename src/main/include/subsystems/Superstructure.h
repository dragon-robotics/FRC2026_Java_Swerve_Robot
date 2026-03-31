#pragma once

#include <functional>
#include <optional>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/hopper/HopperSubsystem.h"
#include "subsystems/intake/IntakeSubsystem.h"
#include "subsystems/shooter/ShooterSubsystem.h"
#include "util/Telemetry.h"

namespace vision {
class VisionSubsystem;
}

class RobotContainer;

class Superstructure : public frc2::SubsystemBase {
 public:
  enum class SuperState {
    DRIVE,
    INTAKE,
    OUTTAKE,
    SHOOT,
    SHOOT_JUICER
  };

  Superstructure(CommandSwerveDrivetrain* swerve,
                 IntakeSubsystem* intake,
                 HopperSubsystem* hopper,
                 ShooterSubsystem* shooter,
                 vision::VisionSubsystem* vision,
                 RobotContainer* container);

  // Heading accessors (used by DefaultDriveCmd)
  std::optional<frc::Rotation2d> GetCurrentHeading() const {
    return m_currentHeading;
  }
  void SetCurrentHeading(std::optional<frc::Rotation2d> heading) {
    m_currentHeading = heading;
  }
  double GetRotationLastTriggered() const { return m_rotationLastTriggered; }
  void SetRotationLastTriggered(double t) { m_rotationLastTriggered = t; }

  // Drive commands
  frc2::CommandPtr DefaultDrive(std::function<double()> translationSup,
                                std::function<double()> strafeSup,
                                std::function<double()> rotationSup,
                                std::function<bool()> halfSpeedSup);

  frc2::CommandPtr ShootDrive(std::function<double()> translationSup,
                              std::function<double()> strafeSup);

  frc2::CommandPtr AimAtTargetPose();
  frc2::CommandPtr SwerveBrakeCmd();
  frc2::CommandPtr SeedFieldCentricCmd();

  // Intake commands
  frc2::CommandPtr IntakeCommand();
  frc2::CommandPtr OuttakeCommand();
  frc2::CommandPtr DeployIntakeCommand();
  frc2::CommandPtr StowIntakeCommand();
  frc2::CommandPtr WokTossIntakeCommand();

  // Hopper commands
  frc2::CommandPtr IndexToIntakeCommand();
  frc2::CommandPtr IndexToShooterCommand();
  frc2::CommandPtr StopHopperCommand();

  // Shooter commands
  frc2::CommandPtr StopShooterCommand();
  frc2::CommandPtr ShootCommand();
  frc2::CommandPtr PrepFuelCommand();

  // SuperState commands
  frc2::CommandPtr DriveSuperstateCommand();

  // Alignment
  bool IsAlignedToTarget() const { return m_alignedToTarget; }

  // State machine
  void SetDesiredSuperState(SuperState state) { m_state = state; }
  void HandleStateTransition();

  void Periodic() override;

 private:
  void UpdateAlignmentStatus(frc::Pose2d const& currentPose,
                             frc::Translation2d const& hubTarget);

  CommandSwerveDrivetrain* m_swerve;
  IntakeSubsystem* m_intake;
  HopperSubsystem* m_hopper;
  ShooterSubsystem* m_shooter;
  vision::VisionSubsystem* m_vision;
  RobotContainer* m_container;

  Telemetry m_logger;

  ctre::phoenix6::swerve::requests::SwerveDriveBrake m_brake;
  ctre::phoenix6::swerve::requests::PointWheelsAt m_point;
  ctre::phoenix6::swerve::requests::ApplyFieldSpeeds m_applyFieldSpeeds;
  ctre::phoenix6::swerve::requests::ApplyRobotSpeeds m_applyRobotSpeeds;

  std::optional<frc::Rotation2d> m_currentHeading;
  double m_rotationLastTriggered = 0.0;

  static constexpr double kAlignmentToleranceDegrees = 3.0;
  bool m_alignedToTarget = false;
  std::optional<frc::Translation2d> m_cachedHubTarget;

  SuperState m_state = SuperState::DRIVE;
  std::optional<SuperState> m_lastState;
};
