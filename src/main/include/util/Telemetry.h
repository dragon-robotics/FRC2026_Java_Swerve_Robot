#pragma once

#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>

class Telemetry {
 public:
  Telemetry() = default;

  void Telemeterize(
      ctre::phoenix6::swerve::SwerveDrivetrain<
          ctre::phoenix6::hardware::TalonFX,
          ctre::phoenix6::hardware::TalonFX,
          ctre::phoenix6::hardware::CANcoder>::SwerveDriveState const& state);
};
