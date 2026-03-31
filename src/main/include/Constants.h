#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/current.h>
#include <units/voltage.h>
#include <units/time.h>
#include <units/mass.h>

#include <frc/RobotBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/Filesystem.h>
#include <frc/DriverStation.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Rotation3d.h>

#include <wpi/interpolating_map.h>
#include <Eigen/Core>

#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include <array>
#include <string>
#include <cmath>

namespace GeneralConstants {
  enum class RobotMode {
    TEST,
    COMP,
    SIM
  };

  inline RobotMode GetCurrentMode() {
    return frc::RobotBase::IsReal() ? RobotMode::COMP : RobotMode::SIM;
  }
}  // namespace GeneralConstants

namespace SwerveConstants {
  inline constexpr double kSteerKP = 100.0;
  inline constexpr double kSteerKI = 0.0;
  inline constexpr double kSteerKD = 0.5;
  inline constexpr double kSteerKS = 0.1;
  inline constexpr double kSteerKV = 1.59;
  inline constexpr double kSteerKA = 0.0;

  inline constexpr double kDriveKP = 0.1;
  inline constexpr double kDriveKI = 0.0;
  inline constexpr double kDriveKD = 0.0;
  inline constexpr double kDriveKS = 0.0;
  inline constexpr double kDriveKV = 0.124;
  inline constexpr double kDriveKA = 0.0;

  inline constexpr double kHeadingKP = 8.0;
  inline constexpr double kHeadingKI = 0.0;
  inline constexpr double kHeadingKD = 0.3;
  inline constexpr double kHeadingTolerance = 0.01;

  inline constexpr double kAngleGearRatio = 12.8;
  inline constexpr double kDriveGearRatio = 6.12;
  inline constexpr double kPulsePerRotation = 1.0;
  inline constexpr double kWheelDiameterInches = 4.0;
  inline constexpr double kWheelDiameterMeters = kWheelDiameterInches * 0.0254;
  inline constexpr double kMaxSpeedFeetPerSecond = 18.2;
  inline constexpr double kMaxSpeedMetersPerSecond = kMaxSpeedFeetPerSecond * 0.3048;

  inline constexpr double kRobotMass = (148.0 - 20.3) * 0.453592;
  inline constexpr double kChassisMass = kRobotMass;
  inline const frc::Translation3d kChassisCG{0_m, 0_m,
      units::meter_t{8.0 * 0.0254}};
  inline constexpr double kLoopTime = 0.13;

  inline constexpr double kWheelLockTime = 10.0;
  inline constexpr double kSwerveDeadband = 0.1;

  // Odometry standard deviations
  inline constexpr double kOdometryStdX = 12.0 * 0.0254;      // inches to meters
  inline constexpr double kOdometryStdY = 12.0 * 0.0254;
  inline constexpr double kOdometryStdTheta = 3.0 * std::numbers::pi / 180.0;  // degrees to radians
}  // namespace SwerveConstants

namespace OperatorConstants {
  inline constexpr int kDriverPort = 0;
  inline constexpr int kOperatorPort = 1;
  inline constexpr int kOperatorButtonPort = 2;
  inline constexpr int kTestPort = 3;
}  // namespace OperatorConstants

namespace JoystickConstants {
  inline constexpr int kStickLeftX = 0;
  inline constexpr int kStickLeftY = 1;
  inline constexpr int kTriggerLeft = 2;
  inline constexpr int kTriggerRight = 3;
  inline constexpr int kStickRightX = 4;
  inline constexpr int kStickRightY = 5;

  inline constexpr int kBtnA = 1;
  inline constexpr int kBtnB = 2;
  inline constexpr int kBtnX = 3;
  inline constexpr int kBtnY = 4;
  inline constexpr int kBumperLeft = 5;
  inline constexpr int kBumperRight = 6;
  inline constexpr int kBtnBack = 7;
  inline constexpr int kBtnStart = 8;
  inline constexpr int kBtnStickLeft = 9;
  inline constexpr int kBtnStickRight = 10;
}  // namespace JoystickConstants

namespace ShooterConstants {
  inline constexpr int kShooterHoodMotorId = 13;
  inline constexpr int kShooterKickerMotorId = 14;
  inline constexpr int kShooterLeadMotorId = 15;
  inline constexpr int kShooterFollowMotorId = 16;
  inline constexpr int kShooterCancoderId = 2;

  inline constexpr double kShooterKickerDutyCycle = 1.0;
  inline constexpr double kShooterKickerVoltage = 12.0;
  inline constexpr double kShooterKickerRPM = 3000.0;
  inline constexpr double kShooterLeadDutyCycle = 1.0;
  inline constexpr double kShooterLeadVoltage = 12.0;
  inline constexpr double kShooterLeadRPM = 2500.0;
  inline constexpr double kShooterHoodSetting = 0.0;

  struct ShooterSetpoint {
    double shooterRPM;
    double hoodAngle;
  };

  inline wpi::interpolating_map<double, double> MakeShooterRPMMap() {
    wpi::interpolating_map<double, double> map;
    map.insert(5.0 * 0.3048, 2450.0);
    map.insert(6.0 * 0.3048, 2500.0);
    map.insert(7.0 * 0.3048, 2550.0);
    map.insert(8.0 * 0.3048, 2700.0);
    map.insert(9.0 * 0.3048, 2800.0);
    map.insert(10.0 * 0.3048, 2850.0);
    map.insert(11.0 * 0.3048, 2900.0);
    map.insert(12.0 * 0.3048, 3000.0);
    return map;
  }

  inline wpi::interpolating_map<double, double> MakeShooterHoodMap() {
    wpi::interpolating_map<double, double> map;
    map.insert(5.0 * 0.3048, 0.00);
    map.insert(6.0 * 0.3048, 0.00);
    map.insert(7.0 * 0.3048, 0.00);
    map.insert(8.0 * 0.3048, 0.00);
    map.insert(9.0 * 0.3048, 0.00);
    map.insert(10.0 * 0.3048, 0.75);
    map.insert(11.0 * 0.3048, 0.75);
    map.insert(12.0 * 0.3048, 1.25);
    return map;
  }

  inline ShooterSetpoint GetSetpointForDistance(double distanceMeters) {
    static auto rpmMap = MakeShooterRPMMap();
    static auto hoodMap = MakeShooterHoodMap();
    return ShooterSetpoint{rpmMap[distanceMeters], hoodMap[distanceMeters]};
  }

  inline ctre::phoenix6::configs::TalonFXConfiguration MakeShooterLeadTalonFXConfig() {
    ctre::phoenix6::configs::TalonFXConfiguration cfg{};
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 40_A;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 30_A;
    cfg.CurrentLimits.SupplyCurrentLowerLimit = 20_A;
    cfg.CurrentLimits.SupplyCurrentLowerTime = 1_s;
    cfg.Voltage.PeakForwardVoltage = 12_V;
    cfg.Voltage.PeakReverseVoltage = -12_V;
    cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    cfg.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    cfg.Slot0.kP = 8.0;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    cfg.Slot0.kS = 4.325;
    cfg.Slot0.kV = 0.013;
    cfg.Slot0.kA = 0.0;
    return cfg;
  }

  inline ctre::phoenix6::configs::TalonFXConfiguration MakeShooterFollowTalonFXConfig() {
    ctre::phoenix6::configs::TalonFXConfiguration cfg{};
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 40_A;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 30_A;
    cfg.CurrentLimits.SupplyCurrentLowerLimit = 20_A;
    cfg.CurrentLimits.SupplyCurrentLowerTime = 1_s;
    cfg.Voltage.PeakForwardVoltage = 12_V;
    cfg.Voltage.PeakReverseVoltage = -12_V;
    cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    return cfg;
  }

  inline ctre::phoenix6::configs::TalonFXConfiguration MakeShooterKickerTalonFXConfig() {
    ctre::phoenix6::configs::TalonFXConfiguration cfg{};
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 60_A;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40_A;
    cfg.CurrentLimits.SupplyCurrentLowerLimit = 30_A;
    cfg.CurrentLimits.SupplyCurrentLowerTime = 1_s;
    cfg.Voltage.PeakForwardVoltage = 11_V;
    cfg.Voltage.PeakReverseVoltage = -11_V;
    cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    cfg.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    return cfg;
  }

  inline ctre::phoenix6::configs::TalonFXConfiguration MakeShooterHoodTalonFXConfig() {
    ctre::phoenix6::configs::TalonFXConfiguration cfg{};
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 30_A;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 20_A;
    cfg.Voltage.PeakForwardVoltage = 10_V;
    cfg.Voltage.PeakReverseVoltage = -10_V;
    cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    cfg.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    cfg.Slot0.kP = 8.0;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.1;
    cfg.Slot0.kS = 0.0;
    cfg.Slot0.kV = 0.0;
    cfg.Slot0.kA = 0.0;
    cfg.Slot0.kG = 0.4;
    cfg.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
    cfg.Slot0.StaticFeedforwardSign = ctre::phoenix6::signals::StaticFeedforwardSignValue::UseClosedLoopSign;
    return cfg;
  }

  inline ctre::phoenix6::configs::CANcoderConfiguration MakeShooterHoodCancoderConfig() {
    ctre::phoenix6::configs::CANcoderConfiguration cfg{};
    cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    cfg.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    cfg.MagnetSensor.MagnetOffset = 0.0;
    return cfg;
  }
}  // namespace ShooterConstants

namespace IntakeConstants {
  inline constexpr int kIntakeRollerMotorId = 12;
  inline constexpr int kIntakeArmMotorId = 11;
  inline constexpr int kIntakeArmCancoderId = 0;
  inline constexpr int kIntakeRollerCancoderId = 1;

  inline constexpr int kIntakeArmFastPidSlot = 0;
  inline constexpr int kIntakeArmSlowPidSlot = 1;

  inline constexpr double kIntakeArmLengthMeters = 13.370 * 0.0254;
  inline constexpr double kIntakeArmMassKg = 10.0 * 0.453592;
  inline constexpr double kIntakeArmGearRatio = 36.0;
  inline constexpr double kIntakeMinAngleRadians = 0.0;
  inline constexpr double kIntakeMaxAngleRadians = 90.0 * std::numbers::pi / 180.0;
  inline constexpr double kIntakeStartingAngleRadians = kIntakeMinAngleRadians;

  inline constexpr double kIntakeRollerDutyCycle = 1.0;
  inline constexpr double kIntakeRollerVoltage = 12.0;
  inline constexpr double kIntakeRollerRPM = 6000.0;
  inline constexpr double kOuttakeRollerDutyCycle = -1.0;
  inline constexpr double kOuttakeRollerVoltage = -12.0;
  inline constexpr double kOuttakeRollerRPM = -6000.0;

  inline constexpr double kIntakeArmStowedPosition = 0.37;
  inline constexpr double kIntakeArmWoktossPosition = 0.15;
  inline constexpr double kIntakeArmJuicerPrePosition = 0.15;
  inline constexpr double kIntakeArmJuicerFinalPosition = 0.28;
  inline constexpr double kIntakeArmDeployedPosition = 0.0;
  inline constexpr double kIntakeArmPositionTolerance = 0.025;

  inline ctre::phoenix6::configs::TalonFXConfiguration MakeIntakeRollerTalonFXConfig() {
    ctre::phoenix6::configs::TalonFXConfiguration cfg{};
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 60_A;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40_A;
    cfg.CurrentLimits.SupplyCurrentLowerLimit = 20_A;
    cfg.CurrentLimits.SupplyCurrentLowerTime = 1_s;
    cfg.Voltage.PeakForwardVoltage = 11_V;
    cfg.Voltage.PeakReverseVoltage = -11_V;
    cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1_s;
    cfg.OpenLoopRamps.TorqueOpenLoopRampPeriod = 0.1_s;
    cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1_s;
    cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    cfg.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    return cfg;
  }

  inline ctre::phoenix6::configs::TalonFXConfiguration MakeIntakeArmTalonFXConfig() {
    ctre::phoenix6::configs::TalonFXConfiguration cfg{};
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 30_A;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 25_A;
    cfg.CurrentLimits.SupplyCurrentLowerLimit = 20_A;
    cfg.CurrentLimits.SupplyCurrentLowerTime = 1_s;
    cfg.Voltage.PeakForwardVoltage = 10_V;
    cfg.Voltage.PeakReverseVoltage = -10_V;
    cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1_s;
    cfg.OpenLoopRamps.TorqueOpenLoopRampPeriod = 0.1_s;
    cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1_s;
    cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    cfg.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    // Slot0 - Fast profile
    cfg.Slot0.kP = 15.0;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    cfg.Slot0.kS = 0.0;
    cfg.Slot0.kV = 2.4;
    cfg.Slot0.kA = 0.0;
    cfg.Slot0.kG = 0.45;
    cfg.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
    cfg.Slot0.StaticFeedforwardSign = ctre::phoenix6::signals::StaticFeedforwardSignValue::UseClosedLoopSign;
    // Slot1 - Slow profile
    cfg.Slot1.kP = 7.0;
    cfg.Slot1.kI = 0.0;
    cfg.Slot1.kD = 0.0;
    cfg.Slot1.kS = 0.0;
    cfg.Slot1.kV = 2.4;
    cfg.Slot1.kA = 0.0;
    cfg.Slot1.kG = 0.45;
    cfg.Slot1.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
    cfg.Slot1.StaticFeedforwardSign = ctre::phoenix6::signals::StaticFeedforwardSignValue::UseClosedLoopSign;
    // Motion Magic
    cfg.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    cfg.MotionMagic.MotionMagicAcceleration = 200.0;
    cfg.MotionMagic.MotionMagicJerk = 200.0;
    cfg.MotionMagic.MotionMagicExpo_kV = 2.0;
    cfg.MotionMagic.MotionMagicExpo_kA = 2.0;
    // Feedback - FusedCANcoder
    cfg.Feedback.FeedbackRemoteSensorID = kIntakeArmCancoderId;
    cfg.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
    cfg.Feedback.SensorToMechanismRatio = 1.0;
    cfg.Feedback.RotorToSensorRatio = kIntakeArmGearRatio;
    cfg.Feedback.FeedbackRotorOffset = 0.0;
    return cfg;
  }

  inline ctre::phoenix6::configs::CANcoderConfiguration MakeIntakeArmCancoderConfig() {
    ctre::phoenix6::configs::CANcoderConfiguration cfg{};
    cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    cfg.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    cfg.MagnetSensor.MagnetOffset = 0.015869546875;
    return cfg;
  }
}  // namespace IntakeConstants

namespace HopperConstants {
  inline constexpr int kHopperRollerLeadMotorId = 17;
  inline constexpr int kHopperRollerFollowMotorId = 18;
  inline constexpr double kHopperRollerStatorCurrentLimit = 31.0;
  inline constexpr double kHopperRollerSupplyCurrentLimit = 20.0;
  inline constexpr double kHopperRollerMaxVoltage = 10.0;
  inline constexpr double kHopperRollerRampRate = 0.25;
  inline constexpr double kHopperRollerDutyCycle = 1.0;
  inline constexpr double kHopperRollerRPM = 1000.0;
  inline constexpr double kHopperRollerReverseRPM = -1000.0;

  inline ctre::phoenix6::configs::TalonFXConfiguration MakeHopperRollerLeadTalonFXConfig() {
    ctre::phoenix6::configs::TalonFXConfiguration cfg{};
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{kHopperRollerStatorCurrentLimit};
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = units::current::ampere_t{kHopperRollerSupplyCurrentLimit};
    cfg.Voltage.PeakForwardVoltage = units::voltage::volt_t{kHopperRollerMaxVoltage};
    cfg.Voltage.PeakReverseVoltage = units::voltage::volt_t{-kHopperRollerMaxVoltage};
    cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = units::time::second_t{kHopperRollerRampRate};
    cfg.OpenLoopRamps.TorqueOpenLoopRampPeriod = units::time::second_t{kHopperRollerRampRate};
    cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = units::time::second_t{kHopperRollerRampRate};
    cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    cfg.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    return cfg;
  }

  inline ctre::phoenix6::configs::TalonFXConfiguration MakeHopperRollerFollowTalonFXConfig() {
    ctre::phoenix6::configs::TalonFXConfiguration cfg{};
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{kHopperRollerStatorCurrentLimit};
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = units::current::ampere_t{kHopperRollerSupplyCurrentLimit};
    cfg.Voltage.PeakForwardVoltage = units::voltage::volt_t{kHopperRollerMaxVoltage};
    cfg.Voltage.PeakReverseVoltage = units::voltage::volt_t{-kHopperRollerMaxVoltage};
    cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = units::time::second_t{kHopperRollerRampRate};
    cfg.OpenLoopRamps.TorqueOpenLoopRampPeriod = units::time::second_t{kHopperRollerRampRate};
    cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = units::time::second_t{kHopperRollerRampRate};
    cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    cfg.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    return cfg;
  }
}  // namespace HopperConstants

namespace VisionConstants {
  inline const std::array<std::string, 4> kApTagCameraNames = {
    "AprilTagPoseEstCameraF",
    "AprilTagPoseEstCameraR",
    "AprilTagPoseEstCameraB",
    "AprilTagPoseEstCameraL"
  };

  inline const frc::Transform3d kApTagPoseEstCamFPos{
      frc::Translation3d{units::meter_t{-11.4 * 0.0254},
                         units::meter_t{-7.7 * 0.0254},
                         units::meter_t{21.25 * 0.0254}},
      frc::Rotation3d{0_rad, units::radian_t{-15.0 * std::numbers::pi / 180.0}, 0_rad}};

  inline const frc::Transform3d kApTagPoseEstCamRPos{
      frc::Translation3d{units::meter_t{-8.64 * 0.0254},
                         units::meter_t{-13.35 * 0.0254},
                         units::meter_t{15.9 * 0.0254}},
      frc::Rotation3d{0_rad, units::radian_t{-15.0 * std::numbers::pi / 180.0},
                      units::radian_t{-90.0 * std::numbers::pi / 180.0}}};

  inline const frc::Transform3d kApTagPoseEstCamBPos{
      frc::Translation3d{units::meter_t{-9.42 * 0.0254},
                         units::meter_t{12.5 * 0.0254},
                         units::meter_t{20.84 * 0.0254}},
      frc::Rotation3d{0_rad, units::radian_t{-15.0 * std::numbers::pi / 180.0},
                      units::radian_t{180.0 * std::numbers::pi / 180.0}}};

  inline const frc::Transform3d kApTagPoseEstCamLPos{
      frc::Translation3d{units::meter_t{-8.64 * 0.0254},
                         units::meter_t{13.36 * 0.0254},
                         units::meter_t{15.93 * 0.0254}},
      frc::Rotation3d{0_rad, units::radian_t{-15.0 * std::numbers::pi / 180.0},
                      units::radian_t{90.0 * std::numbers::pi / 180.0}}};

  inline const std::array<frc::Transform3d, 4> kApTagPoseEstCamPositions = {
    kApTagPoseEstCamFPos,
    kApTagPoseEstCamRPos,
    kApTagPoseEstCamBPos,
    kApTagPoseEstCamLPos
  };

  // Standard deviations for vision measurements
  inline constexpr double kSingleTagStdDevX = 4.0;
  inline constexpr double kSingleTagStdDevY = 4.0;
  inline constexpr double kSingleTagStdDevTheta = 8.0;
  inline constexpr double kMultiTagStdDevX = 0.5;
  inline constexpr double kMultiTagStdDevY = 0.5;
  inline constexpr double kMultiTagStdDevTheta = 1.0;
  inline constexpr double kDefaultTagStdDevX = 0.9;
  inline constexpr double kDefaultTagStdDevY = 0.9;
  inline constexpr double kDefaultTagStdDevTheta = 0.9;

  inline constexpr double kMaxAmbiguity = 0.1;
  inline constexpr double kMaxZError = 1.5 * 0.0254;  // inches to meters

  inline constexpr double kMaxPoseDiscrepancyMeters = 1.0;

  inline constexpr double kLinearStdDevBaseline = 0.3;
  inline constexpr double kAngularStdDevBaseline = 10.0 * std::numbers::pi / 180.0;

  inline constexpr double kCameraFovHorizontalDegrees = 73.0;
  inline constexpr double kCameraAspectRatioWidth = 4.0;
  inline constexpr double kCameraAspectRatioHeight = 3.0;

  inline double CalculateVerticalFOV(double horizontalFOV, double aspectWidth, double aspectHeight) {
    double hFovRad = horizontalFOV * std::numbers::pi / 180.0;
    double tanHFovHalf = std::tan(hFovRad / 2.0);
    double tanVFovHalf = tanHFovHalf * (aspectHeight / aspectWidth);
    double vFovHalfRad = std::atan(tanVFovHalf);
    return vFovHalfRad * 2.0 * 180.0 / std::numbers::pi;
  }

  inline const double kCameraFovVerticalDegrees = CalculateVerticalFOV(
      kCameraFovHorizontalDegrees, kCameraAspectRatioWidth, kCameraAspectRatioHeight);

  inline constexpr double kLinearStdDevMegatag2Factor = 0.5;
  inline constexpr double kAngularStdDevMegatag2AngleFactor =
      std::numeric_limits<double>::infinity();
}  // namespace VisionConstants

namespace FieldConstants {
  inline frc::AprilTagFieldLayout LoadFieldLayout() {
    std::string path = frc::filesystem::GetDeployDirectory() +
                       "/apriltags/welded/2026-rebuilt-welded.json";
    try {
      return frc::AprilTagFieldLayout{path};
    } catch (...) {
      frc::DriverStation::ReportError(
          "CRITICAL: Failed to load default AprilTag field resource!");
      return frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);
    }
  }

  inline frc::AprilTagFieldLayout& GetFieldLayout() {
    static frc::AprilTagFieldLayout layout = LoadFieldLayout();
    return layout;
  }

  inline double GetFieldLength() { return GetFieldLayout().GetFieldLength().value(); }
  inline double GetFieldWidth() { return GetFieldLayout().GetFieldWidth().value(); }
  inline constexpr double kFieldHeight = 72.0 * 0.0254;

  namespace Hub {
    inline frc::Translation2d GetBlueHubCenterPose() {
      auto& layout = GetFieldLayout();
      auto tag20 = layout.GetTagPose(20);
      auto tag26 = layout.GetTagPose(26);
      double x = (tag20->GetX().value() + tag26->GetX().value()) / 2.0;
      double y = tag20->GetY().value();
      return frc::Translation2d{units::meter_t{x}, units::meter_t{y}};
    }

    inline frc::Translation2d GetRedHubCenterPose() {
      auto& layout = GetFieldLayout();
      auto tag4 = layout.GetTagPose(4);
      auto tag10 = layout.GetTagPose(10);
      double x = (tag4->GetX().value() + tag10->GetX().value()) / 2.0;
      double y = tag4->GetY().value();
      return frc::Translation2d{units::meter_t{x}, units::meter_t{y}};
    }
  }  // namespace Hub
}  // namespace FieldConstants
