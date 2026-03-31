#include "subsystems/CommandSwerveDrivetrain.h"

#include <frc/DriverStation.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/PIDConstants.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <ctre/phoenix6/SignalLogger.hpp>

using namespace ctre::phoenix6;

CommandSwerveDrivetrain::CommandSwerveDrivetrain(
    swerve::SwerveDrivetrainConstants const& drivetrainConstants,
    swerve::SwerveModuleConstants<
        configs::TalonFXConfiguration,
        configs::TalonFXConfiguration,
        configs::CANcoderConfiguration> const&... modules)
    : swerve::SwerveDrivetrain<hardware::TalonFX, hardware::TalonFX, hardware::CANcoder>(
          drivetrainConstants, modules...),
      m_sysIdRoutineTranslation{
          frc2::sysid::SysIdRoutine::Config{
              std::nullopt,            // Default ramp rate (1 V/s)
              4_V,                     // Step voltage
              std::nullopt,            // Default timeout (10 s)
              [](frc::sysid::State state) {
                SignalLogger::WriteString("SysIdTranslation_State",
                                         frc::sysid::SysIdRoutineLog::StateEnumToString(state));
              }},
          frc2::sysid::SysIdRoutine::Mechanism{
              [this](units::volt_t output) {
                SetControl(m_translationCharacterization.WithVolts(output));
              },
              nullptr,
              this}},
      m_sysIdRoutineSteer{
          frc2::sysid::SysIdRoutine::Config{
              std::nullopt,
              7_V,
              std::nullopt,
              [](frc::sysid::State state) {
                SignalLogger::WriteString("SysIdSteer_State",
                                         frc::sysid::SysIdRoutineLog::StateEnumToString(state));
              }},
          frc2::sysid::SysIdRoutine::Mechanism{
              [this](units::volt_t output) {
                SetControl(m_steerCharacterization.WithVolts(output));
              },
              nullptr,
              this}},
      m_sysIdRoutineRotation{
          frc2::sysid::SysIdRoutine::Config{
              units::volt_t{std::numbers::pi / 6.0} / 1_s,
              units::volt_t{std::numbers::pi},
              std::nullopt,
              [](frc::sysid::State state) {
                SignalLogger::WriteString("SysIdRotation_State",
                                         frc::sysid::SysIdRoutineLog::StateEnumToString(state));
              }},
          frc2::sysid::SysIdRoutine::Mechanism{
              [this](units::volt_t output) {
                SetControl(m_rotationCharacterization.WithRotationalRate(output.value()));
                SignalLogger::WriteDouble("Rotational_Rate", output.value());
              },
              nullptr,
              this}},
      m_sysIdRoutineToApply{&m_sysIdRoutineTranslation} {
  if (Utils::IsSimulation()) {
    StartSimThread();
  }
  ConfigureAutoBuilder();
}

CommandSwerveDrivetrain::CommandSwerveDrivetrain(
    swerve::SwerveDrivetrainConstants const& drivetrainConstants,
    units::frequency::hertz_t odometryUpdateFrequency,
    swerve::SwerveModuleConstants<
        configs::TalonFXConfiguration,
        configs::TalonFXConfiguration,
        configs::CANcoderConfiguration> const&... modules)
    : swerve::SwerveDrivetrain<hardware::TalonFX, hardware::TalonFX, hardware::CANcoder>(
          drivetrainConstants, odometryUpdateFrequency, modules...),
      m_sysIdRoutineTranslation{
          frc2::sysid::SysIdRoutine::Config{
              std::nullopt, 4_V, std::nullopt,
              [](frc::sysid::State state) {
                SignalLogger::WriteString("SysIdTranslation_State",
                                         frc::sysid::SysIdRoutineLog::StateEnumToString(state));
              }},
          frc2::sysid::SysIdRoutine::Mechanism{
              [this](units::volt_t output) {
                SetControl(m_translationCharacterization.WithVolts(output));
              },
              nullptr, this}},
      m_sysIdRoutineSteer{
          frc2::sysid::SysIdRoutine::Config{
              std::nullopt, 7_V, std::nullopt,
              [](frc::sysid::State state) {
                SignalLogger::WriteString("SysIdSteer_State",
                                         frc::sysid::SysIdRoutineLog::StateEnumToString(state));
              }},
          frc2::sysid::SysIdRoutine::Mechanism{
              [this](units::volt_t output) {
                SetControl(m_steerCharacterization.WithVolts(output));
              },
              nullptr, this}},
      m_sysIdRoutineRotation{
          frc2::sysid::SysIdRoutine::Config{
              units::volt_t{std::numbers::pi / 6.0} / 1_s,
              units::volt_t{std::numbers::pi}, std::nullopt,
              [](frc::sysid::State state) {
                SignalLogger::WriteString("SysIdRotation_State",
                                         frc::sysid::SysIdRoutineLog::StateEnumToString(state));
              }},
          frc2::sysid::SysIdRoutine::Mechanism{
              [this](units::volt_t output) {
                SetControl(m_rotationCharacterization.WithRotationalRate(output.value()));
                SignalLogger::WriteDouble("Rotational_Rate", output.value());
              },
              nullptr, this}},
      m_sysIdRoutineToApply{&m_sysIdRoutineTranslation} {
  if (Utils::IsSimulation()) {
    StartSimThread();
  }
  ConfigureAutoBuilder();
}

CommandSwerveDrivetrain::CommandSwerveDrivetrain(
    swerve::SwerveDrivetrainConstants const& drivetrainConstants,
    units::frequency::hertz_t odometryUpdateFrequency,
    Eigen::Matrix<double, 3, 1> const& odometryStandardDeviation,
    Eigen::Matrix<double, 3, 1> const& visionStandardDeviation,
    swerve::SwerveModuleConstants<
        configs::TalonFXConfiguration,
        configs::TalonFXConfiguration,
        configs::CANcoderConfiguration> const&... modules)
    : swerve::SwerveDrivetrain<hardware::TalonFX, hardware::TalonFX, hardware::CANcoder>(
          drivetrainConstants, odometryUpdateFrequency,
          odometryStandardDeviation, visionStandardDeviation, modules...),
      m_sysIdRoutineTranslation{
          frc2::sysid::SysIdRoutine::Config{
              std::nullopt, 4_V, std::nullopt,
              [](frc::sysid::State state) {
                SignalLogger::WriteString("SysIdTranslation_State",
                                         frc::sysid::SysIdRoutineLog::StateEnumToString(state));
              }},
          frc2::sysid::SysIdRoutine::Mechanism{
              [this](units::volt_t output) {
                SetControl(m_translationCharacterization.WithVolts(output));
              },
              nullptr, this}},
      m_sysIdRoutineSteer{
          frc2::sysid::SysIdRoutine::Config{
              std::nullopt, 7_V, std::nullopt,
              [](frc::sysid::State state) {
                SignalLogger::WriteString("SysIdSteer_State",
                                         frc::sysid::SysIdRoutineLog::StateEnumToString(state));
              }},
          frc2::sysid::SysIdRoutine::Mechanism{
              [this](units::volt_t output) {
                SetControl(m_steerCharacterization.WithVolts(output));
              },
              nullptr, this}},
      m_sysIdRoutineRotation{
          frc2::sysid::SysIdRoutine::Config{
              units::volt_t{std::numbers::pi / 6.0} / 1_s,
              units::volt_t{std::numbers::pi}, std::nullopt,
              [](frc::sysid::State state) {
                SignalLogger::WriteString("SysIdRotation_State",
                                         frc::sysid::SysIdRoutineLog::StateEnumToString(state));
              }},
          frc2::sysid::SysIdRoutine::Mechanism{
              [this](units::volt_t output) {
                SetControl(m_rotationCharacterization.WithRotationalRate(output.value()));
                SignalLogger::WriteDouble("Rotational_Rate", output.value());
              },
              nullptr, this}},
      m_sysIdRoutineToApply{&m_sysIdRoutineTranslation} {
  if (Utils::IsSimulation()) {
    StartSimThread();
  }
  ConfigureAutoBuilder();
}

void CommandSwerveDrivetrain::ConfigureAutoBuilder() {
  try {
    auto config = pathplanner::RobotConfig::fromGUISettings();
    pathplanner::AutoBuilder::configure(
        [this] { return GetState().Pose; },
        [this](frc::Pose2d const& pose) { ResetPose(pose); },
        [this] { return GetState().Speeds; },
        [this](frc::ChassisSpeeds const& speeds,
               pathplanner::DriveFeedforwards const& feedforwards) {
          SetControl(
              m_pathApplyRobotSpeeds
                  .WithSpeeds(frc::ChassisSpeeds::Discretize(speeds, 0.020_s))
                  .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                  .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY));
        },
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            pathplanner::PIDConstants{10.0, 0.0, 0.0},
            pathplanner::PIDConstants{7.0, 0.0, 0.0}),
        config,
        [] {
          auto alliance = frc::DriverStation::GetAlliance();
          return alliance.has_value() &&
                 alliance.value() == frc::DriverStation::Alliance::kRed;
        },
        this);
  } catch (std::exception const& ex) {
    frc::DriverStation::ReportError(
        std::string("Failed to load PathPlanner config and configure AutoBuilder: ") + ex.what());
  }
}

frc2::CommandPtr CommandSwerveDrivetrain::ApplyRequest(
    std::function<ctre::phoenix6::swerve::requests::SwerveRequest const&()> request) {
  return Run([this, req = std::move(request)] { SetControl(req()); });
}

frc2::CommandPtr CommandSwerveDrivetrain::SysIdQuasistatic(frc2::sysid::Direction direction) {
  return m_sysIdRoutineToApply->Quasistatic(direction);
}

frc2::CommandPtr CommandSwerveDrivetrain::SysIdDynamic(frc2::sysid::Direction direction) {
  return m_sysIdRoutineToApply->Dynamic(direction);
}

void CommandSwerveDrivetrain::Periodic() {
  if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value()) {
      SetOperatorPerspectiveForward(
          alliance.value() == frc::DriverStation::Alliance::kRed
              ? kRedAlliancePerspectiveRotation
              : kBlueAlliancePerspectiveRotation);
      m_hasAppliedOperatorPerspective = true;
    }
  }
}

void CommandSwerveDrivetrain::StartSimThread() {
  m_lastSimTime = units::second_t{Utils::GetCurrentTimeSeconds()};

  m_simNotifier = std::make_unique<frc::Notifier>([this] {
    auto currentTime = units::second_t{Utils::GetCurrentTimeSeconds()};
    auto deltaTime = currentTime - m_lastSimTime;
    m_lastSimTime = currentTime;

    UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
  });
  m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

void CommandSwerveDrivetrain::AddVisionMeasurement(
    frc::Pose2d const& visionRobotPose, units::second_t timestamp) {
  swerve::SwerveDrivetrain<hardware::TalonFX, hardware::TalonFX, hardware::CANcoder>::
      AddVisionMeasurement(visionRobotPose,
                           units::second_t{Utils::FpgaToCurrentTime(timestamp.value())});
}

void CommandSwerveDrivetrain::AddVisionMeasurement(
    frc::Pose2d const& visionRobotPose,
    units::second_t timestamp,
    Eigen::Matrix<double, 3, 1> const& visionStdDevs) {
  swerve::SwerveDrivetrain<hardware::TalonFX, hardware::TalonFX, hardware::CANcoder>::
      AddVisionMeasurement(visionRobotPose,
                           units::second_t{Utils::FpgaToCurrentTime(timestamp.value())},
                           visionStdDevs);
}

std::optional<frc::Pose2d> CommandSwerveDrivetrain::SamplePoseAt(units::second_t timestamp) {
  return swerve::SwerveDrivetrain<hardware::TalonFX, hardware::TalonFX, hardware::CANcoder>::
      SamplePoseAt(units::second_t{Utils::FpgaToCurrentTime(timestamp.value())});
}
