#pragma once

#include <functional>
#include <optional>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/Notifier.h>
#include <frc/RobotController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include <ctre/phoenix6/SignalLogger.hpp>
#include <ctre/phoenix6/Utils.hpp>
#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>
#include <ctre/phoenix6/swerve/SwerveDrivetrainConstants.hpp>
#include <ctre/phoenix6/swerve/SwerveModuleConstants.hpp>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>

#include <Eigen/Core>

class CommandSwerveDrivetrain
    : public ctre::phoenix6::swerve::SwerveDrivetrain<
          ctre::phoenix6::hardware::TalonFX,
          ctre::phoenix6::hardware::TalonFX,
          ctre::phoenix6::hardware::CANcoder>,
      public frc2::SubsystemBase {
 public:
  CommandSwerveDrivetrain(
      ctre::phoenix6::swerve::SwerveDrivetrainConstants const& drivetrainConstants,
      ctre::phoenix6::swerve::SwerveModuleConstants<
          ctre::phoenix6::configs::TalonFXConfiguration,
          ctre::phoenix6::configs::TalonFXConfiguration,
          ctre::phoenix6::configs::CANcoderConfiguration> const&... modules);

  CommandSwerveDrivetrain(
      ctre::phoenix6::swerve::SwerveDrivetrainConstants const& drivetrainConstants,
      units::frequency::hertz_t odometryUpdateFrequency,
      ctre::phoenix6::swerve::SwerveModuleConstants<
          ctre::phoenix6::configs::TalonFXConfiguration,
          ctre::phoenix6::configs::TalonFXConfiguration,
          ctre::phoenix6::configs::CANcoderConfiguration> const&... modules);

  CommandSwerveDrivetrain(
      ctre::phoenix6::swerve::SwerveDrivetrainConstants const& drivetrainConstants,
      units::frequency::hertz_t odometryUpdateFrequency,
      Eigen::Matrix<double, 3, 1> const& odometryStandardDeviation,
      Eigen::Matrix<double, 3, 1> const& visionStandardDeviation,
      ctre::phoenix6::swerve::SwerveModuleConstants<
          ctre::phoenix6::configs::TalonFXConfiguration,
          ctre::phoenix6::configs::TalonFXConfiguration,
          ctre::phoenix6::configs::CANcoderConfiguration> const&... modules);

  frc2::CommandPtr ApplyRequest(
      std::function<ctre::phoenix6::swerve::requests::SwerveRequest const&()> request);

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

  void Periodic() override;

  void AddVisionMeasurement(frc::Pose2d const& visionRobotPose,
                            units::second_t timestamp);
  void AddVisionMeasurement(frc::Pose2d const& visionRobotPose,
                            units::second_t timestamp,
                            Eigen::Matrix<double, 3, 1> const& visionStdDevs);

  std::optional<frc::Pose2d> SamplePoseAt(units::second_t timestamp);

 private:
  void ConfigureAutoBuilder();
  void StartSimThread();

  static constexpr units::second_t kSimLoopPeriod = 0.004_s;

  static inline const frc::Rotation2d kBlueAlliancePerspectiveRotation{};
  static inline const frc::Rotation2d kRedAlliancePerspectiveRotation{units::degree_t{180}};

  bool m_hasAppliedOperatorPerspective = false;

  ctre::phoenix6::swerve::requests::ApplyRobotSpeeds m_pathApplyRobotSpeeds{};

  ctre::phoenix6::swerve::requests::SysIdSwerveTranslation m_translationCharacterization{};
  ctre::phoenix6::swerve::requests::SysIdSwerveSteerGains m_steerCharacterization{};
  ctre::phoenix6::swerve::requests::SysIdSwerveRotation m_rotationCharacterization{};

  frc2::sysid::SysIdRoutine m_sysIdRoutineTranslation;
  frc2::sysid::SysIdRoutine m_sysIdRoutineSteer;
  frc2::sysid::SysIdRoutine m_sysIdRoutineRotation;
  frc2::sysid::SysIdRoutine* m_sysIdRoutineToApply;

  std::unique_ptr<frc::Notifier> m_simNotifier;
  units::second_t m_lastSimTime{0_s};
};
