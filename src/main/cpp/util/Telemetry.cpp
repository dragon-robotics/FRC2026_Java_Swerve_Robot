#include "util/Telemetry.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Telemetry::Telemeterize(
    ctre::phoenix6::swerve::SwerveDrivetrain<
        ctre::phoenix6::hardware::TalonFX,
        ctre::phoenix6::hardware::TalonFX,
        ctre::phoenix6::hardware::CANcoder>::SwerveDriveState const& state) {
  // Log current robot pose
  frc::DataLogManager::Log(
      "Pose: " + state.Pose.ToString());
  frc::SmartDashboard::PutString("Robot Pose", state.Pose.ToString());
}
