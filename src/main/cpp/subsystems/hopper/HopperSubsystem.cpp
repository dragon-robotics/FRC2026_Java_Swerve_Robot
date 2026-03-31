#include "subsystems/hopper/HopperSubsystem.h"

#include "Constants.h"

using namespace HopperConstants;

HopperSubsystem::HopperSubsystem(std::unique_ptr<MotorIO> leadRollerMotorIO,
                                 std::unique_ptr<MotorIO> followRollerMotorIO)
    : m_leadRollerMotorIO{std::move(leadRollerMotorIO)},
      m_followRollerMotorIO{std::move(followRollerMotorIO)},
      m_currHopperState{HopperState::STOP},
      m_desiredHopperState{HopperState::STOP},
      m_lastCommandedState{HopperState::STOP} {}

void HopperSubsystem::RunHopperRollerRPM(double rpm) {
  m_leadRollerMotorIO->SetMotorRPM(rpm);
}

void HopperSubsystem::RunHopperRollerVoltage(double voltage) {
  m_leadRollerMotorIO->SetMotorVoltage(voltage);
}

void HopperSubsystem::RunHopperRollerPercentage(double percentage) {
  m_leadRollerMotorIO->SetMotorPercentage(percentage);
}

void HopperSubsystem::IndexToShooter() {
  m_leadRollerMotorIO->SetMotorPercentage(kHopperRollerDutyCycle);
}

void HopperSubsystem::IndexToIntake() {
  m_leadRollerMotorIO->SetMotorPercentage(-kHopperRollerDutyCycle);
}

void HopperSubsystem::StopHopperRoller() {
  m_leadRollerMotorIO->SetMotorPercentage(0.0);
}

void HopperSubsystem::SetDesiredState(HopperState state) {
  m_currHopperState = state;
}

void HopperSubsystem::HandleStateTransition() {
  // Skip redundant CAN writes if state hasn't changed
  if (m_lastCommandedStateValid && m_currHopperState == m_lastCommandedState) {
    return;
  }
  m_lastCommandedState = m_currHopperState;
  m_lastCommandedStateValid = true;

  switch (m_currHopperState) {
    case HopperState::STOP:
      StopHopperRoller();
      break;
    case HopperState::INDEXTOSHOOTER:
      IndexToShooter();
      break;
    case HopperState::INDEXTOINTAKE:
      IndexToIntake();
      break;
  }
}

void HopperSubsystem::Periodic() {
  HandleStateTransition();
}
