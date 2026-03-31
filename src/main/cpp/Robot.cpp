// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/RobotController.h>
#include <frc2/command/CommandScheduler.h>

Robot::Robot() : frc::TimedRobot{0.025_s} {
  m_robotContainer = std::make_unique<RobotContainer>();
}

void Robot::RobotPeriodic() {
  // Batch-refresh all CTRE motor signals BEFORE the scheduler runs subsystem
  // periodics
  SignalRegistry::GetInstance().RefreshAll();

  frc2::CommandScheduler::GetInstance().Run();

  // Power monitoring
  frc::RobotController::GetBatteryVoltage();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_robotContainer->GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand);
  }
}

void Robot::AutonomousPeriodic() {}
void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand != nullptr) {
    frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand);
  }
}

void Robot::TeleopPeriodic() {}
void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}
void Robot::TestExit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
