#include "subsystems/Superstructure.h"

#include <numbers>
#include <cmath>

#include <frc/DriverStation.h>
#include <frc2/command/InstantCommand.h>

#include "commands/AimAtTargetPoseCmd.h"
#include "commands/DefaultDriveCmd.h"
#include "commands/ShootDriveCmd.h"
#include "Constants.h"

Superstructure::Superstructure(CommandSwerveDrivetrain* swerve,
                               IntakeSubsystem* intake,
                               HopperSubsystem* hopper,
                               ShooterSubsystem* shooter,
                               vision::VisionSubsystem* vision,
                               RobotContainer* container)
    : m_swerve{swerve},
      m_intake{intake},
      m_hopper{hopper},
      m_shooter{shooter},
      m_vision{vision},
      m_container{container} {
  m_applyFieldSpeeds.DesaturateWheelSpeeds = true;
  m_applyFieldSpeeds.DriveRequestType =
      ctre::phoenix6::swerve::SwerveModule::DriveRequestType::Velocity;
  m_applyRobotSpeeds.DesaturateWheelSpeeds = true;
  m_applyRobotSpeeds.DriveRequestType =
      ctre::phoenix6::swerve::SwerveModule::DriveRequestType::Velocity;

  m_swerve->RegisterTelemetry(
      [this](auto const& state) { m_logger.Telemeterize(state); });
}

frc2::CommandPtr Superstructure::DefaultDrive(
    std::function<double()> translationSup,
    std::function<double()> strafeSup,
    std::function<double()> rotationSup,
    std::function<bool()> halfSpeedSup) {
  return frc2::CommandPtr{std::make_unique<DefaultDriveCmd>(
      m_swerve,
      std::move(translationSup),
      std::move(strafeSup),
      std::move(rotationSup),
      std::move(halfSpeedSup),
      [this] { return GetCurrentHeading(); },
      [this](auto h) { SetCurrentHeading(h); },
      [this] { return GetRotationLastTriggered(); },
      [this](double t) { SetRotationLastTriggered(t); })};
}

frc2::CommandPtr Superstructure::ShootDrive(std::function<double()> translationSup,
                                            std::function<double()> strafeSup) {
  return frc2::CommandPtr{std::make_unique<ShootDriveCmd>(
      m_swerve,
      std::move(translationSup),
      std::move(strafeSup),
      [this](auto h) { SetCurrentHeading(h); })};
}

frc2::CommandPtr Superstructure::AimAtTargetPose() {
  return frc2::CommandPtr{std::make_unique<AimAtTargetPoseCmd>(
      m_swerve, [this](auto h) { SetCurrentHeading(h); })};
}

frc2::CommandPtr Superstructure::SwerveBrakeCmd() {
  return m_swerve->ApplyRequest([this]() -> ctre::phoenix6::swerve::requests::SwerveRequest const& {
    return m_brake;
  });
}

frc2::CommandPtr Superstructure::SeedFieldCentricCmd() {
  return m_swerve->RunOnce([this] { m_swerve->SeedFieldCentric(); });
}

frc2::CommandPtr Superstructure::IntakeCommand() {
  return frc2::InstantCommand([this] {
           m_intake->SetDesiredState(IntakeSubsystem::IntakeState::INTAKE);
         },
         {m_intake})
      .ToPtr();
}

frc2::CommandPtr Superstructure::OuttakeCommand() {
  return frc2::InstantCommand([this] {
           m_intake->SetDesiredState(IntakeSubsystem::IntakeState::OUTTAKE);
         },
         {m_intake})
      .ToPtr();
}

frc2::CommandPtr Superstructure::DeployIntakeCommand() {
  return frc2::InstantCommand([this] {
           m_intake->SetDesiredState(IntakeSubsystem::IntakeState::DEPLOYED);
         },
         {m_intake})
      .ToPtr();
}

frc2::CommandPtr Superstructure::StowIntakeCommand() {
  return frc2::InstantCommand([this] {
           m_intake->SetDesiredState(IntakeSubsystem::IntakeState::HOME);
         },
         {m_intake})
      .ToPtr();
}

frc2::CommandPtr Superstructure::WokTossIntakeCommand() {
  return frc2::InstantCommand([this] {
           m_intake->SetDesiredState(IntakeSubsystem::IntakeState::WOKTOSS);
         },
         {m_intake})
      .ToPtr();
}

frc2::CommandPtr Superstructure::IndexToIntakeCommand() {
  return frc2::InstantCommand([this] {
           m_hopper->SetDesiredState(HopperSubsystem::HopperState::INDEXTOINTAKE);
         },
         {m_hopper})
      .ToPtr();
}

frc2::CommandPtr Superstructure::IndexToShooterCommand() {
  return frc2::InstantCommand([this] {
           m_hopper->SetDesiredState(HopperSubsystem::HopperState::INDEXTOSHOOTER);
         },
         {m_hopper})
      .ToPtr();
}

frc2::CommandPtr Superstructure::StopHopperCommand() {
  return frc2::InstantCommand([this] {
           m_hopper->SetDesiredState(HopperSubsystem::HopperState::STOP);
         },
         {m_hopper})
      .ToPtr();
}

frc2::CommandPtr Superstructure::StopShooterCommand() {
  return frc2::InstantCommand([this] {
           m_shooter->SetDesiredState(ShooterSubsystem::ShooterState::STOP);
         },
         {m_shooter})
      .ToPtr();
}

frc2::CommandPtr Superstructure::ShootCommand() {
  return frc2::InstantCommand([this] {
           m_shooter->SetDesiredState(ShooterSubsystem::ShooterState::SHOOT);
         },
         {m_shooter})
      .ToPtr();
}

frc2::CommandPtr Superstructure::PrepFuelCommand() {
  return frc2::InstantCommand([this] {
           m_shooter->SetDesiredState(ShooterSubsystem::ShooterState::PREPFUEL);
         },
         {m_shooter})
      .ToPtr();
}

frc2::CommandPtr Superstructure::DriveSuperstateCommand() {
  return frc2::InstantCommand([this] {
           SetDesiredSuperState(SuperState::DRIVE);
         })
      .ToPtr();
}

void Superstructure::UpdateAlignmentStatus(frc::Pose2d const& currentPose,
                                           frc::Translation2d const& hubTarget) {
  double dx = hubTarget.X().value() - currentPose.X().value();
  double dy = hubTarget.Y().value() - currentPose.Y().value();
  double targetAngleRad = std::atan2(dy, dx);

  double headingErrorRad =
      currentPose.Rotation().Radians().value() - targetAngleRad;
  // Normalize to [-pi, pi]
  while (headingErrorRad > std::numbers::pi) headingErrorRad -= 2.0 * std::numbers::pi;
  while (headingErrorRad < -std::numbers::pi) headingErrorRad += 2.0 * std::numbers::pi;

  m_alignedToTarget =
      std::abs(headingErrorRad * 180.0 / std::numbers::pi) < kAlignmentToleranceDegrees;
}

void Superstructure::HandleStateTransition() {
  switch (m_state) {
    case SuperState::DRIVE:
      m_intake->SetDesiredState(IntakeSubsystem::IntakeState::DEPLOYED);
      m_hopper->SetDesiredState(HopperSubsystem::HopperState::STOP);
      m_shooter->SetDesiredState(ShooterSubsystem::ShooterState::STOP);
      break;

    case SuperState::INTAKE:
      m_intake->SetDesiredState(IntakeSubsystem::IntakeState::INTAKE);
      m_shooter->SetDesiredState(ShooterSubsystem::ShooterState::STOP);
      break;

    case SuperState::OUTTAKE:
      m_intake->SetDesiredState(IntakeSubsystem::IntakeState::OUTTAKE);
      m_hopper->SetDesiredState(HopperSubsystem::HopperState::INDEXTOINTAKE);
      m_shooter->SetDesiredState(ShooterSubsystem::ShooterState::STOP);
      break;

    case SuperState::SHOOT:
      m_shooter->SetDesiredState(ShooterSubsystem::ShooterState::SHOOT);
      if (m_shooter->GetCurrentState() == ShooterSubsystem::ShooterState::SHOOT &&
          IsAlignedToTarget()) {
        m_swerve->SetControl(m_brake);
        m_intake->SetDesiredState(IntakeSubsystem::IntakeState::INTAKE);
        m_hopper->SetDesiredState(HopperSubsystem::HopperState::INDEXTOSHOOTER);
      } else {
        m_hopper->SetDesiredState(HopperSubsystem::HopperState::STOP);
        m_intake->SetDesiredState(IntakeSubsystem::IntakeState::DEPLOYED);
      }
      break;

    case SuperState::SHOOT_JUICER:
      m_shooter->SetDesiredState(ShooterSubsystem::ShooterState::SHOOT);
      if (m_shooter->GetCurrentState() == ShooterSubsystem::ShooterState::SHOOT &&
          IsAlignedToTarget()) {
        m_swerve->SetControl(m_brake);
        m_intake->SetDesiredState(IntakeSubsystem::IntakeState::JUICER);
        m_hopper->SetDesiredState(HopperSubsystem::HopperState::INDEXTOSHOOTER);
      } else {
        m_hopper->SetDesiredState(HopperSubsystem::HopperState::STOP);
        m_intake->SetDesiredState(IntakeSubsystem::IntakeState::DEPLOYED);
      }
      break;
  }
}

void Superstructure::Periodic() {
  // Cache the hub target once alliance is known (doesn't change mid-match)
  if (!m_cachedHubTarget.has_value()) {
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value()) {
      m_cachedHubTarget =
          alliance.value() == frc::DriverStation::Alliance::kRed
              ? FieldConstants::Hub::GetRedHubCenterPose()
              : FieldConstants::Hub::GetBlueHubCenterPose();
    }
  }

  if (m_cachedHubTarget.has_value()) {
    auto currentPose = m_swerve->GetState().Pose;

    double distanceToHub =
        currentPose.Translation().Distance(m_cachedHubTarget.value()).value();

    m_shooter->SetSetpointForDistance(distanceToHub);
    UpdateAlignmentStatus(currentPose, m_cachedHubTarget.value());
  }

  HandleStateTransition();
}
