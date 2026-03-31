#include "subsystems/intake/IntakeSubsystem.h"

#include <cmath>

#include "Constants.h"

using namespace IntakeConstants;

IntakeSubsystem::IntakeSubsystem(std::unique_ptr<MotorIO> intakeRollerIO,
                                 std::unique_ptr<MotorIO> intakeArmIO)
    : m_intakeRollerIO{std::move(intakeRollerIO)},
      m_intakeArmIO{std::move(intakeArmIO)},
      m_currIntakeState{IntakeState::HOME},
      m_desiredIntakeState{IntakeState::HOME} {}

void IntakeSubsystem::RunIntake() {
  m_intakeRollerIO->SetMotorPercentage(kIntakeRollerDutyCycle);
}

void IntakeSubsystem::RunOuttake() {
  m_intakeRollerIO->SetMotorPercentage(kOuttakeRollerDutyCycle);
}

void IntakeSubsystem::StopIntake() {
  m_intakeRollerIO->SetMotorPercentage(0.0);
}

void IntakeSubsystem::DeployIntakeArm() {
  m_intakeArmIO->SetMotorPosition(kIntakeArmDeployedPosition, kIntakeArmFastPidSlot);
}

void IntakeSubsystem::CoastIntakeArm() {
  m_intakeArmIO->SetMotorPercentage(0.0);
}

void IntakeSubsystem::StowIntakeArm() {
  m_intakeArmIO->SetMotorPosition(kIntakeArmStowedPosition, kIntakeArmSlowPidSlot);
}

void IntakeSubsystem::WokTossIntakeArm() {
  m_intakeArmIO->SetMotorPosition(kIntakeArmWoktossPosition, kIntakeArmSlowPidSlot);
}

void IntakeSubsystem::SetIntakeArmSetpoint(double setpoint, int slotID) {
  m_intakeArmIO->SetMotorPosition(setpoint, slotID);
}

bool IntakeSubsystem::IsIntakeArmAtDeployed() const {
  return std::abs(kIntakeArmDeployedPosition - m_intakeArmInputs.motorPosition) <
         kIntakeArmPositionTolerance;
}

bool IntakeSubsystem::IsIntakeArmAtStowed() const {
  return std::abs(kIntakeArmStowedPosition - m_intakeArmInputs.motorPosition) <
         kIntakeArmPositionTolerance;
}

bool IntakeSubsystem::IsIntakeArmAtWokToss() const {
  return std::abs(kIntakeArmWoktossPosition - m_intakeArmInputs.motorPosition) <
         kIntakeArmPositionTolerance;
}

bool IntakeSubsystem::IsIntakeArmAtPreJuice() const {
  return std::abs(kIntakeArmJuicerPrePosition - m_intakeArmInputs.motorPosition) <
         kIntakeArmPositionTolerance;
}

void IntakeSubsystem::SetDesiredState(IntakeState state) {
  m_desiredIntakeState = state;

  if (m_currIntakeState == state) {
    return;
  }

  // State is changing — reset guard
  m_lastCommandedStateValid = false;

  switch (m_desiredIntakeState) {
    case IntakeState::HOME:
      m_currIntakeState = IntakeState::STOWING;
      break;
    case IntakeState::INTAKE:
      if (m_currIntakeState == IntakeState::OUTTAKE ||
          m_currIntakeState == IntakeState::DEPLOYED ||
          m_currIntakeState == IntakeState::INTAKE) {
        m_currIntakeState = IntakeState::INTAKE;
      } else {
        m_currIntakeState = IntakeState::DEPLOYING;
      }
      break;
    case IntakeState::OUTTAKE:
      if (m_currIntakeState == IntakeState::INTAKE ||
          m_currIntakeState == IntakeState::DEPLOYED ||
          m_currIntakeState == IntakeState::OUTTAKE) {
        m_currIntakeState = IntakeState::OUTTAKE;
      } else {
        m_currIntakeState = IntakeState::DEPLOYING;
      }
      break;
    case IntakeState::DEPLOYED:
      m_currIntakeState = IntakeState::DEPLOYING;
      break;
    case IntakeState::WOKTOSS:
      m_currIntakeState = IntakeState::WOKTOSSING;
      m_wokTossMovingToDeployed = true;
      break;
    case IntakeState::AUTO_WOKTOSSING:
      m_currIntakeState = IntakeState::AUTO_WOKTOSSING;
      m_wokTossMovingToDeployed = true;
      break;
    case IntakeState::JUICER:
      m_currIntakeState = IntakeState::JUICER;
      m_juicerPhase = JuicerPhase::PRE_JUICE;
      m_lastJuicerPhaseValid = false;
      break;
    default:
      break;
  }
}

void IntakeSubsystem::HandleStateTransition() {
  switch (m_currIntakeState) {
    case IntakeState::HOME:
      if (!m_lastCommandedStateValid || m_lastCommandedState != m_currIntakeState) {
        StowIntakeArm();
        StopIntake();
        m_lastCommandedState = m_currIntakeState;
        m_lastCommandedStateValid = true;
        m_homeCoasting = false;
      } else if (!m_homeCoasting && IsIntakeArmAtStowed()) {
        CoastIntakeArm();
        m_homeCoasting = true;
      }
      break;

    case IntakeState::INTAKE:
      if (!m_lastCommandedStateValid || m_lastCommandedState != m_currIntakeState) {
        CoastIntakeArm();
        RunIntake();
        m_lastCommandedState = m_currIntakeState;
        m_lastCommandedStateValid = true;
      }
      break;

    case IntakeState::OUTTAKE:
      if (!m_lastCommandedStateValid || m_lastCommandedState != m_currIntakeState) {
        CoastIntakeArm();
        RunOuttake();
        m_lastCommandedState = m_currIntakeState;
        m_lastCommandedStateValid = true;
      }
      break;

    case IntakeState::DEPLOYED:
      if (!m_lastCommandedStateValid || m_lastCommandedState != m_currIntakeState) {
        CoastIntakeArm();
        StopIntake();
        m_lastCommandedState = m_currIntakeState;
        m_lastCommandedStateValid = true;
      }
      break;

    case IntakeState::WOKTOSS:
      if (!m_lastCommandedStateValid || m_lastCommandedState != m_currIntakeState) {
        WokTossIntakeArm();
        RunIntake();
        m_lastCommandedState = m_currIntakeState;
        m_lastCommandedStateValid = true;
      }
      break;

    case IntakeState::DEPLOYING:
      if (!m_lastCommandedStateValid || m_lastCommandedState != m_currIntakeState) {
        DeployIntakeArm();
        StopIntake();
        m_lastCommandedState = m_currIntakeState;
        m_lastCommandedStateValid = true;
      }
      if (IsIntakeArmAtDeployed()) {
        m_lastCommandedStateValid = false;
        if (m_desiredIntakeState == IntakeState::INTAKE) {
          m_currIntakeState = IntakeState::INTAKE;
        } else if (m_desiredIntakeState == IntakeState::OUTTAKE) {
          m_currIntakeState = IntakeState::OUTTAKE;
        } else {
          m_currIntakeState = IntakeState::DEPLOYED;
        }
      }
      break;

    case IntakeState::STOWING:
      if (!m_lastCommandedStateValid || m_lastCommandedState != m_currIntakeState) {
        StowIntakeArm();
        StopIntake();
        m_lastCommandedState = m_currIntakeState;
        m_lastCommandedStateValid = true;
      }
      if (IsIntakeArmAtStowed()) {
        m_lastCommandedStateValid = false;
        m_currIntakeState = IntakeState::HOME;
      }
      break;

    case IntakeState::WOKTOSSING:
      if (!m_lastCommandedStateValid || m_lastCommandedState != m_currIntakeState) {
        WokTossIntakeArm();
        RunIntake();
        m_lastCommandedState = m_currIntakeState;
        m_lastCommandedStateValid = true;
      }
      if (IsIntakeArmAtWokToss()) {
        m_lastCommandedStateValid = false;
        m_currIntakeState = IntakeState::WOKTOSS;
      }
      break;

    case IntakeState::AUTO_WOKTOSSING:
      // Placeholder for oscillating arm behavior
      break;

    case IntakeState::JUICER:
      if (!m_lastCommandedStateValid || m_lastCommandedState != m_currIntakeState) {
        RunIntake();
        m_lastCommandedState = m_currIntakeState;
        m_lastCommandedStateValid = true;
        m_lastJuicerPhaseValid = false;
      }
      switch (m_juicerPhase) {
        case JuicerPhase::PRE_JUICE:
          if (!m_lastJuicerPhaseValid || m_lastJuicerPhase != m_juicerPhase) {
            SetIntakeArmSetpoint(kIntakeArmJuicerPrePosition, kIntakeArmFastPidSlot);
            m_lastJuicerPhase = m_juicerPhase;
            m_lastJuicerPhaseValid = true;
          }
          if (IsIntakeArmAtPreJuice()) {
            m_juicerPhase = JuicerPhase::SQUEEZE;
          }
          break;
        case JuicerPhase::SQUEEZE:
          if (!m_lastJuicerPhaseValid || m_lastJuicerPhase != m_juicerPhase) {
            SetIntakeArmSetpoint(kIntakeArmJuicerFinalPosition, kIntakeArmSlowPidSlot);
            m_lastJuicerPhase = m_juicerPhase;
            m_lastJuicerPhaseValid = true;
          }
          break;
      }
      break;
  }
}

void IntakeSubsystem::Periodic() {
  HandleStateTransition();
  m_intakeArmIO->UpdateInputs(m_intakeArmInputs);
}
