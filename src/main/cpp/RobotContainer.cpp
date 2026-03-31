#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/FollowPathCommand.h>

#include "Constants.h"
#include "generated/TunerConstants.h"
#include "io/TalonFXMotorIO.h"
#include "io/TalonFXMotorIOSim.h"
#include "io/TalonFXMotorIOTunable.h"
#include "subsystems/vision/VisionIOPhotonVision.h"
#include "subsystems/vision/VisionIOPhotonVisionSim.h"

using namespace GeneralConstants;
using namespace IntakeConstants;
using namespace HopperConstants;
using namespace ShooterConstants;
using namespace VisionConstants;
using namespace OperatorConstants;
using namespace ctre::phoenix6;

// Type alias for the swerve drive state
using SwerveDriveState =
    ctre::phoenix6::swerve::SwerveDrivetrain<
        ctre::phoenix6::hardware::TalonFX,
        ctre::phoenix6::hardware::TalonFX,
        ctre::phoenix6::hardware::CANcoder>::SwerveDriveState;

RobotContainer::RobotContainer()
    : m_driverController{kDriverPort},
      m_operatorController{kOperatorPort} {

  // Silence joystick connection warnings
  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  // Build the swerve drivetrain using TunerConstants
  Eigen::Matrix<double, 3, 1> odometryStdDevs{
      SwerveConstants::kOdometryStdX,
      SwerveConstants::kOdometryStdY,
      SwerveConstants::kOdometryStdTheta};
  Eigen::Matrix<double, 3, 1> visionStdDevs{
      kDefaultTagStdDevX, kDefaultTagStdDevY, kDefaultTagStdDevTheta};

  m_swerveOwner = std::make_unique<CommandSwerveDrivetrain>(
      TunerConstants::MakeDrivetrainConstants(),
      units::frequency::hertz_t{250},
      odometryStdDevs, visionStdDevs,
      TunerConstants::MakeFrontLeft(),
      TunerConstants::MakeFrontRight(),
      TunerConstants::MakeBackLeft(),
      TunerConstants::MakeBackRight());
  swerveSubsystem = m_swerveOwner.get();

  // Initialize subsystems based on the current mode
  auto mode = GetCurrentMode();

  if (mode == RobotMode::COMP || mode == RobotMode::TEST) {
    bool tunable = (mode == RobotMode::TEST);

    auto makeMotorIO = [&](int id, ctre::phoenix6::configs::TalonFXConfiguration cfg,
                           std::string name) -> std::unique_ptr<MotorIO> {
      if (tunable) {
        return std::make_unique<TalonFXMotorIOTunable>(id, cfg, std::move(name));
      }
      return std::make_unique<TalonFXMotorIO>(id, cfg, std::move(name));
    };

    auto makeMotorIOWithFollower = [&](int id,
                                       ctre::phoenix6::configs::TalonFXConfiguration cfg,
                                       std::string name,
                                       controls::Follower followerCfg) -> std::unique_ptr<MotorIO> {
      if (tunable) {
        return std::make_unique<TalonFXMotorIOTunable>(id, cfg, std::move(name),
                                                       std::move(followerCfg));
      }
      return std::make_unique<TalonFXMotorIO>(id, cfg, std::move(name),
                                              std::move(followerCfg));
    };

    m_intakeOwner = std::make_unique<IntakeSubsystem>(
        makeMotorIO(kIntakeRollerMotorId, MakeIntakeRollerTalonFXConfig(), "Intake Roller"),
        makeMotorIO(kIntakeArmMotorId, MakeIntakeArmTalonFXConfig(), "Intake Arm"));

    m_hopperOwner = std::make_unique<HopperSubsystem>(
        makeMotorIO(kHopperRollerLeadMotorId, MakeHopperRollerLeadTalonFXConfig(),
                    "Hopper Lead Motor"),
        makeMotorIOWithFollower(
            kHopperRollerFollowMotorId, MakeHopperRollerFollowTalonFXConfig(),
            "Hopper Follow Motor",
            controls::Follower{kHopperRollerLeadMotorId, false}));

    m_shooterOwner = std::make_unique<ShooterSubsystem>(
        makeMotorIO(kShooterLeadMotorId, MakeShooterLeadTalonFXConfig(), "Shooter Lead"),
        makeMotorIOWithFollower(
            kShooterFollowMotorId, MakeShooterFollowTalonFXConfig(), "Shooter Follow",
            controls::Follower{kShooterLeadMotorId, true}),
        makeMotorIO(kShooterKickerMotorId, MakeShooterKickerTalonFXConfig(), "Shooter Kicker"),
        makeMotorIO(kShooterHoodMotorId, MakeShooterHoodTalonFXConfig(), "Shooter Hood"));

    std::vector<std::unique_ptr<vision::VisionIO>> visionIOs;
    for (size_t i = 0; i < kApTagCameraNames.size(); ++i) {
      visionIOs.push_back(std::make_unique<vision::VisionIOPhotonVision>(
          kApTagCameraNames[i],
          kApTagPoseEstCamPositions[i],
          [this]() -> SwerveDriveState { return swerveSubsystem->GetState(); }));
    }

    auto visionConsumer =
        [this](frc::Pose2d const& pose, double timestamp,
               Eigen::Vector3d const& stdDevs) {
          Eigen::Matrix<double, 3, 1> stdDevMat{stdDevs.x(), stdDevs.y(), stdDevs.z()};
          swerveSubsystem->AddVisionMeasurement(pose, units::second_t{timestamp}, stdDevMat);
        };

    m_visionOwner = std::make_unique<vision::VisionSubsystem>(
        swerveSubsystem, std::move(visionConsumer), std::move(visionIOs));

  } else {
    // SIM mode
    m_intakeOwner = std::make_unique<IntakeSubsystem>(
        std::make_unique<TalonFXMotorIOSim>(kIntakeRollerMotorId,
                                            MakeIntakeRollerTalonFXConfig(),
                                            "KrakenX60", "Intake Roller"),
        std::make_unique<TalonFXMotorIOSim>(kIntakeArmMotorId,
                                            MakeIntakeArmTalonFXConfig(),
                                            "KrakenX60", "Intake Arm",
                                            MakeIntakeArmCancoderConfig()));

    m_hopperOwner = std::make_unique<HopperSubsystem>(
        std::make_unique<TalonFXMotorIOSim>(kHopperRollerLeadMotorId,
                                            MakeHopperRollerLeadTalonFXConfig(),
                                            "KrakenX60", "Hopper Lead Motor"),
        std::make_unique<TalonFXMotorIOSim>(
            kHopperRollerFollowMotorId, MakeHopperRollerFollowTalonFXConfig(),
            "KrakenX60", "Hopper Follow Motor",
            controls::Follower{kHopperRollerLeadMotorId, true}));

    m_shooterOwner = std::make_unique<ShooterSubsystem>(
        std::make_unique<TalonFXMotorIOSim>(kShooterLeadMotorId,
                                            MakeShooterLeadTalonFXConfig(),
                                            "KrakenX60_FOC", "Shooter Lead"),
        std::make_unique<TalonFXMotorIOSim>(
            kShooterFollowMotorId, MakeShooterFollowTalonFXConfig(),
            "KrakenX60_FOC", "Shooter Follow",
            controls::Follower{kShooterLeadMotorId, true}),
        std::make_unique<TalonFXMotorIOSim>(kShooterKickerMotorId,
                                            MakeShooterKickerTalonFXConfig(),
                                            "KrakenX60_FOC", "Shooter Kicker"),
        std::make_unique<TalonFXMotorIOSim>(kShooterHoodMotorId,
                                            MakeShooterHoodTalonFXConfig(),
                                            "KrakenX44", "Shooter Hood"));

    std::vector<std::unique_ptr<vision::VisionIO>> visionIOs;
    for (size_t i = 0; i < kApTagCameraNames.size(); ++i) {
      visionIOs.push_back(std::make_unique<vision::VisionIOPhotonVisionSim>(
          kApTagCameraNames[i],
          kApTagPoseEstCamPositions[i],
          [this]() -> SwerveDriveState { return swerveSubsystem->GetState(); }));
    }

    auto visionConsumer =
        [this](frc::Pose2d const& pose, double timestamp,
               Eigen::Vector3d const& stdDevs) {
          Eigen::Matrix<double, 3, 1> stdDevMat{stdDevs.x(), stdDevs.y(), stdDevs.z()};
          swerveSubsystem->AddVisionMeasurement(pose, units::second_t{timestamp}, stdDevMat);
        };

    m_visionOwner = std::make_unique<vision::VisionSubsystem>(
        swerveSubsystem, std::move(visionConsumer), std::move(visionIOs));
  }

  intakeSubsystem = m_intakeOwner.get();
  hopperSubsystem = m_hopperOwner.get();
  shooterSubsystem = m_shooterOwner.get();
  visionSubsystem = m_visionOwner.get();

  // Create superstructure
  m_superstructureOwner = std::make_unique<Superstructure>(
      swerveSubsystem, intakeSubsystem, hopperSubsystem, shooterSubsystem,
      nullptr, this);
  superstructureSubsystem = m_superstructureOwner.get();

  // Build auto chooser
  m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
  frc::SmartDashboard::PutData("Auto Mode", &m_autoChooser);

  ConfigureBindings();

  // Warmup PathPlanner to avoid pauses
  frc2::CommandScheduler::GetInstance().Schedule(
      pathplanner::FollowPathCommand::warmupCommand());
}

void RobotContainer::ConfigureBindings() {
  // Idle while the robot is disabled
  static ctre::phoenix6::swerve::requests::Idle idleRequest{};
  frc2::RobotModeTriggers::Disabled().WhileTrue(
      swerveSubsystem
          ->ApplyRequest(
              []() -> ctre::phoenix6::swerve::requests::SwerveRequest const& {
                return idleRequest;
              })
          .IgnoringDisable(true));

  // Default drive command
  swerveSubsystem->SetDefaultCommand(
      superstructureSubsystem->DefaultDrive(
          [this] { return -m_driverController.GetLeftY(); },
          [this] { return -m_driverController.GetLeftX(); },
          [this] { return -m_driverController.GetRightX(); },
          [this] { return m_driverController.GetHID().GetPOV() == 0; }));

  // Reset field-centric heading on start button press
  m_driverController.Start().OnTrue(superstructureSubsystem->SeedFieldCentricCmd());

  // Intake
  m_driverController.LeftTrigger(0.2)
      .WhileTrue(frc2::InstantCommand([this] {
                   superstructureSubsystem->SetDesiredSuperState(
                       Superstructure::SuperState::INTAKE);
                 },
                 {superstructureSubsystem})
                     .ToPtr())
      .OnFalse(frc2::InstantCommand([this] {
                 superstructureSubsystem->SetDesiredSuperState(
                     Superstructure::SuperState::DRIVE);
               },
               {superstructureSubsystem})
                   .ToPtr());

  // Outtake
  m_driverController.RightBumper()
      .WhileTrue(frc2::InstantCommand([this] {
                   superstructureSubsystem->SetDesiredSuperState(
                       Superstructure::SuperState::OUTTAKE);
                 },
                 {superstructureSubsystem})
                     .ToPtr())
      .OnFalse(frc2::InstantCommand([this] {
                 superstructureSubsystem->SetDesiredSuperState(
                     Superstructure::SuperState::DRIVE);
               },
               {superstructureSubsystem})
                   .ToPtr());

  // Shoot
  m_driverController.RightTrigger(0.2)
      .WhileTrue(frc2::InstantCommand([this] {
                   superstructureSubsystem->SetDesiredSuperState(
                       Superstructure::SuperState::SHOOT);
                 },
                 {superstructureSubsystem})
                     .ToPtr())
      .WhileTrue(superstructureSubsystem->AimAtTargetPose())
      .OnFalse(frc2::InstantCommand([this] {
                 superstructureSubsystem->SetDesiredSuperState(
                     Superstructure::SuperState::DRIVE);
               },
               {superstructureSubsystem})
                   .ToPtr());

  // Shoot Juicer
  m_operatorController.B().WhileTrue(
      frc2::InstantCommand([this] {
        superstructureSubsystem->SetDesiredSuperState(
            Superstructure::SuperState::SHOOT_JUICER);
      },
      {superstructureSubsystem})
          .ToPtr());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoChooser.GetSelected();
}
