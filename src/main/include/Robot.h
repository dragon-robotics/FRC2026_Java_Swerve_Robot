#pragma once

#include <memory>

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>

#include "RobotContainer.h"
#include "io/SignalRegistry.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();

  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;
  void SimulationPeriodic() override;

 private:
  std::unique_ptr<RobotContainer> m_robotContainer;
  frc2::Command* m_autonomousCommand = nullptr;
};
