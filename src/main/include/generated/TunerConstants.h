#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/moment_of_inertia.h>
#include <units/time.h>

#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>
#include <ctre/phoenix6/swerve/SwerveDrivetrainConstants.hpp>
#include <ctre/phoenix6/swerve/SwerveModuleConstants.hpp>
#include <ctre/phoenix6/swerve/SwerveModuleConstantsFactory.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/CANBus.hpp>

namespace TunerConstants {

// Steer motor gains
inline ctre::phoenix6::configs::Slot0Configs MakeSteerGains() {
  ctre::phoenix6::configs::Slot0Configs gains{};
  gains.kP = 100;
  gains.kI = 0;
  gains.kD = 0.5;
  gains.kS = 0.1;
  gains.kV = 1.50;
  gains.kA = 0;
  gains.StaticFeedforwardSign =
      ctre::phoenix6::signals::StaticFeedforwardSignValue::UseClosedLoopSign;
  return gains;
}

// Drive motor gains
inline ctre::phoenix6::configs::Slot0Configs MakeDriveGains() {
  ctre::phoenix6::configs::Slot0Configs gains{};
  gains.kP = 0.1;
  gains.kI = 0;
  gains.kD = 0;
  gains.kS = 0;
  gains.kV = 0.124;
  return gains;
}

// Closed-loop output types
inline constexpr auto kSteerClosedLoopOutput =
    ctre::phoenix6::swerve::ClosedLoopOutputType::Voltage;
inline constexpr auto kDriveClosedLoopOutput =
    ctre::phoenix6::swerve::ClosedLoopOutputType::Voltage;

// Motor arrangement
inline constexpr auto kDriveMotorType =
    ctre::phoenix6::swerve::DriveMotorArrangement::TalonFX_Integrated;
inline constexpr auto kSteerMotorType =
    ctre::phoenix6::swerve::SteerMotorArrangement::TalonFX_Integrated;
inline constexpr auto kSteerFeedbackType =
    ctre::phoenix6::swerve::SteerFeedbackType::FusedCANcoder;

// Slip current
inline constexpr units::current::ampere_t kSlipCurrent = 70_A;

// Drive initial configs
inline ctre::phoenix6::configs::TalonFXConfiguration MakeDriveInitialConfigs() {
  ctre::phoenix6::configs::TalonFXConfiguration cfg{};
  cfg.CurrentLimits.StatorCurrentLimitEnable = true;
  cfg.CurrentLimits.StatorCurrentLimit = 40_A;
  cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
  cfg.CurrentLimits.SupplyCurrentLimit = 30_A;
  cfg.CurrentLimits.SupplyCurrentLowerLimit = 35_A;
  cfg.CurrentLimits.SupplyCurrentLowerTime = 1_s;
  cfg.Voltage.PeakForwardVoltage = 10_V;
  cfg.Voltage.PeakReverseVoltage = -10_V;
  cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2_s;
  cfg.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2_s;
  cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2_s;
  cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  return cfg;
}

// Steer initial configs
inline ctre::phoenix6::configs::TalonFXConfiguration MakeSteerInitialConfigs() {
  ctre::phoenix6::configs::TalonFXConfiguration cfg{};
  cfg.CurrentLimits.StatorCurrentLimitEnable = true;
  cfg.CurrentLimits.StatorCurrentLimit = 40_A;
  cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
  cfg.CurrentLimits.SupplyCurrentLimit = 30_A;
  cfg.Voltage.PeakForwardVoltage = 12_V;
  cfg.Voltage.PeakReverseVoltage = -12_V;
  cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2_s;
  cfg.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2_s;
  cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2_s;
  cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  return cfg;
}

// CAN bus
inline const ctre::phoenix6::CANBus kCANBus{"Swerve", "./logs/example.hoot"};

// Theoretical free speed at 12V
inline constexpr units::velocity::meters_per_second_t kSpeedAt12Volts = 4.76_mps;

// Coupling ratio
inline constexpr double kCoupleRatio = 5.4;

// Gear ratios
inline constexpr double kDriveGearRatio = 6.48;
inline constexpr double kSteerGearRatio = 12.1;
inline constexpr units::length::inch_t kWheelRadius = 2_in;

// Inversion
inline constexpr bool kInvertLeftSide = false;
inline constexpr bool kInvertRightSide = true;

// Pigeon
inline constexpr int kPigeonId = 0;

// Simulation
inline constexpr units::moment_of_inertia::kilogram_square_meter_t kSteerInertia = 0.01_kg_sq_m;
inline constexpr units::moment_of_inertia::kilogram_square_meter_t kDriveInertia = 0.01_kg_sq_m;
inline constexpr units::voltage::volt_t kSteerFrictionVoltage = 0.2_V;
inline constexpr units::voltage::volt_t kDriveFrictionVoltage = 0.2_V;

// Drivetrain constants
inline ctre::phoenix6::swerve::SwerveDrivetrainConstants MakeDrivetrainConstants() {
  ctre::phoenix6::swerve::SwerveDrivetrainConstants constants{};
  constants.CANBusName = kCANBus.GetName();
  constants.Pigeon2Id = kPigeonId;
  return constants;
}

// Module constants factory
inline auto MakeConstantCreator() {
  return ctre::phoenix6::swerve::SwerveModuleConstantsFactory<
      ctre::phoenix6::configs::TalonFXConfiguration,
      ctre::phoenix6::configs::TalonFXConfiguration,
      ctre::phoenix6::configs::CANcoderConfiguration>{}
      .WithDriveMotorGearRatio(kDriveGearRatio)
      .WithSteerMotorGearRatio(kSteerGearRatio)
      .WithCouplingGearRatio(kCoupleRatio)
      .WithWheelRadius(kWheelRadius)
      .WithSteerMotorGains(MakeSteerGains())
      .WithDriveMotorGains(MakeDriveGains())
      .WithSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
      .WithDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
      .WithSlipCurrent(kSlipCurrent)
      .WithSpeedAt12Volts(kSpeedAt12Volts)
      .WithDriveMotorType(kDriveMotorType)
      .WithSteerMotorType(kSteerMotorType)
      .WithFeedbackSource(kSteerFeedbackType)
      .WithDriveMotorInitialConfigs(MakeDriveInitialConfigs())
      .WithSteerMotorInitialConfigs(MakeSteerInitialConfigs())
      .WithSteerInertia(kSteerInertia)
      .WithDriveInertia(kDriveInertia)
      .WithSteerFrictionVoltage(kSteerFrictionVoltage)
      .WithDriveFrictionVoltage(kDriveFrictionVoltage);
}

// Front Left
inline constexpr int kFrontLeftDriveMotorId = 1;
inline constexpr int kFrontLeftSteerMotorId = 2;
inline constexpr int kFrontLeftEncoderId = 1;
inline constexpr units::angle::turn_t kFrontLeftEncoderOffset{-0.324951171875};
inline constexpr bool kFrontLeftSteerMotorInverted = true;
inline constexpr bool kFrontLeftEncoderInverted = false;
inline constexpr units::length::inch_t kFrontLeftXPos = 10.75_in;
inline constexpr units::length::inch_t kFrontLeftYPos = 10.75_in;

// Front Right
inline constexpr int kFrontRightDriveMotorId = 3;
inline constexpr int kFrontRightSteerMotorId = 4;
inline constexpr int kFrontRightEncoderId = 2;
inline constexpr units::angle::turn_t kFrontRightEncoderOffset{0.2744140625};
inline constexpr bool kFrontRightSteerMotorInverted = true;
inline constexpr bool kFrontRightEncoderInverted = false;
inline constexpr units::length::inch_t kFrontRightXPos = 10.75_in;
inline constexpr units::length::inch_t kFrontRightYPos = -10.75_in;

// Back Left
inline constexpr int kBackLeftDriveMotorId = 7;
inline constexpr int kBackLeftSteerMotorId = 8;
inline constexpr int kBackLeftEncoderId = 4;
inline constexpr units::angle::turn_t kBackLeftEncoderOffset{-0.416259765625};
inline constexpr bool kBackLeftSteerMotorInverted = true;
inline constexpr bool kBackLeftEncoderInverted = false;
inline constexpr units::length::inch_t kBackLeftXPos = -10.75_in;
inline constexpr units::length::inch_t kBackLeftYPos = 10.75_in;

// Back Right
inline constexpr int kBackRightDriveMotorId = 5;
inline constexpr int kBackRightSteerMotorId = 6;
inline constexpr int kBackRightEncoderId = 3;
inline constexpr units::angle::turn_t kBackRightEncoderOffset{0.12109375};
inline constexpr bool kBackRightSteerMotorInverted = true;
inline constexpr bool kBackRightEncoderInverted = false;
inline constexpr units::length::inch_t kBackRightXPos = -10.75_in;
inline constexpr units::length::inch_t kBackRightYPos = -10.75_in;

// Module constants - created at startup
inline auto MakeFrontLeft() {
  return MakeConstantCreator().CreateModuleConstants(
      kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
      kFrontLeftEncoderOffset, kFrontLeftXPos, kFrontLeftYPos,
      kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted);
}

inline auto MakeFrontRight() {
  return MakeConstantCreator().CreateModuleConstants(
      kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
      kFrontRightEncoderOffset, kFrontRightXPos, kFrontRightYPos,
      kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted);
}

inline auto MakeBackLeft() {
  return MakeConstantCreator().CreateModuleConstants(
      kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId,
      kBackLeftEncoderOffset, kBackLeftXPos, kBackLeftYPos,
      kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted);
}

inline auto MakeBackRight() {
  return MakeConstantCreator().CreateModuleConstants(
      kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
      kBackRightEncoderOffset, kBackRightXPos, kBackRightYPos,
      kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted);
}

}  // namespace TunerConstants
