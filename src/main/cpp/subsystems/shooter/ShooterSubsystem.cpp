#include "subsystems/shooter/ShooterSubsystem.h"

#include <cmath>

#include "Constants.h"

using namespace ShooterConstants;

ShooterSubsystem::ShooterSubsystem(std::unique_ptr<MotorIO> shooterLeadIO,
                                   std::unique_ptr<MotorIO> shooterFollowIO,
                                   std::unique_ptr<MotorIO> shooterKickerIO,
                                   std::unique_ptr<MotorIO> shooterHoodIO)
    : m_shooterLeadIO{std::move(shooterLeadIO)},
      m_shooterFollowIO{std::move(shooterFollowIO)},
      m_shooterKickerIO{std::move(shooterKickerIO)},
      m_shooterHoodIO{std::move(shooterHoodIO)},
      m_desiredShooterState{ShooterState::STOP},
      m_currShooterState{ShooterState::STOP},
      m_targetRPM{kShooterLeadRPM},
      m_hoodAngle{kShooterHoodSetting} {}

void ShooterSubsystem::RunShooterMotorPercentage(double percentage) {
  m_shooterLeadIO->SetMotorPercentage(percentage);
}

void ShooterSubsystem::RunKickerMotorPercentage(double percentage) {
  m_shooterKickerIO->SetMotorPercentage(percentage);
}

void ShooterSubsystem::RunShooter() {
  m_shooterLeadIO->SetMotorRPM(m_targetRPM);
}

void ShooterSubsystem::PrepShooter() {
  m_shooterLeadIO->SetMotorRPM(m_targetRPM * 0.3);
}

void ShooterSubsystem::StopShooter() {
  m_shooterLeadIO->SetMotorRPM(0.0);
}

void ShooterSubsystem::RunKicker() {
  m_shooterKickerIO->SetMotorPercentage(0.75);
}

void ShooterSubsystem::PrepKicker() {
  m_shooterKickerIO->SetMotorPercentage(0.15);
}

void ShooterSubsystem::StopKicker() {
  m_shooterKickerIO->SetMotorPercentage(0.0);
}

void ShooterSubsystem::SetHoodAngle(double position) {
  m_shooterHoodIO->SetMotorPosition(position);
}

void ShooterSubsystem::SetSetpointForDistance(double distanceToTarget) {
  auto setpoint = GetSetpointForDistance(distanceToTarget);
  m_targetRPM = setpoint.shooterRPM;
  m_hoodAngle = setpoint.hoodAngle;
}

double ShooterSubsystem::GetShooterSpeed() const {
  return m_shooterLeadInputs.motorVelocity * 60.0;  // RPS to RPM
}

bool ShooterSubsystem::IsShooting() const {
  return GetShooterSpeed() > 60.0;
}

bool ShooterSubsystem::IsShooterStopped() const {
  return std::abs(GetShooterSpeed()) < 0.5;
}

void ShooterSubsystem::SetDesiredState(ShooterState state) {
  m_desiredShooterState = state;

  if (m_desiredShooterState != m_currShooterState) {
    switch (m_desiredShooterState) {
      case ShooterState::STOP:
      case ShooterState::PREPFUEL:
      case ShooterState::SHOOT:
        m_currShooterState = ShooterState::TRANSITION;
        break;
      default:
        break;
    }
  }
}

void ShooterSubsystem::HandleStateTransition() {
  switch (m_currShooterState) {
    case ShooterState::STOP:
      if (m_kickerStopTimerRunning) {
        if (m_kickerStopTimer.HasElapsed(units::second_t{kKickerStopDelay})) {
          StopKicker();
          m_kickerStopTimerRunning = false;
          m_kickerStopTimer.Stop();
          m_kickerStopTimer.Reset();
        } else {
          RunKicker();
        }
      }
      break;

    case ShooterState::PREPFUEL:
      m_kickerStopTimerRunning = false;
      m_kickerStopTimer.Stop();
      m_kickerStopTimer.Reset();
      PrepShooter();
      PrepKicker();
      break;

    case ShooterState::SHOOT:
      RunShooter();
      RunKicker();
      SetHoodAngle(m_hoodAngle);
      break;

    case ShooterState::TRANSITION:
      switch (m_desiredShooterState) {
        case ShooterState::STOP:
          StopShooter();
          if (!m_kickerStopTimerRunning) {
            m_kickerStopTimer.Reset();
            m_kickerStopTimer.Start();
            m_kickerStopTimerRunning = true;
          }
          if (m_kickerStopTimerRunning &&
              !m_kickerStopTimer.HasElapsed(units::second_t{kKickerStopDelay})) {
            RunKickerMotorPercentage(0.5);
          } else {
            StopKicker();
          }
          SetHoodAngle(0.0);
          if (IsShooterStopped()) {
            m_currShooterState = ShooterState::STOP;
          }
          break;

        case ShooterState::PREPFUEL:
          PrepShooter();
          PrepKicker();
          SetHoodAngle(0.0);
          if (std::abs(m_targetRPM * 0.3 - GetShooterSpeed()) < 60.0) {
            m_currShooterState = ShooterState::PREPFUEL;
          }
          break;

        case ShooterState::SHOOT:
          RunShooter();
          SetHoodAngle(m_hoodAngle);
          if (std::abs(m_targetRPM - GetShooterSpeed()) < 60.0 &&
              std::abs(m_hoodAngle - m_shooterHoodInputs.motorPosition) < 0.125) {
            m_currShooterState = ShooterState::SHOOT;
          }
          break;

        default:
          break;
      }
      break;
  }
}

void ShooterSubsystem::Periodic() {
  m_shooterLeadIO->UpdateInputs(m_shooterLeadInputs);
  m_shooterHoodIO->UpdateInputs(m_shooterHoodInputs);

  HandleStateTransition();
}
