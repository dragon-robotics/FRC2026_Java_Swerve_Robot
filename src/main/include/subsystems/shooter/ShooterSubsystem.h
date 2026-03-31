#pragma once

#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

#include "io/MotorIO.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  enum class ShooterState {
    STOP,
    PREPFUEL,
    SHOOT,
    TRANSITION
  };

  ShooterSubsystem(std::unique_ptr<MotorIO> shooterLeadIO,
                   std::unique_ptr<MotorIO> shooterFollowIO,
                   std::unique_ptr<MotorIO> shooterKickerIO,
                   std::unique_ptr<MotorIO> shooterHoodIO);

  // Motor control
  void RunShooterMotorPercentage(double percentage);
  void RunKickerMotorPercentage(double percentage);
  void RunShooter();
  void PrepShooter();
  void StopShooter();
  void RunKicker();
  void PrepKicker();
  void StopKicker();
  void SetHoodAngle(double position);
  void SetSetpointForDistance(double distanceToTarget);

  // Getters
  ShooterState GetCurrentState() const { return m_currShooterState; }
  ShooterState GetShooterState() const { return m_desiredShooterState; }
  double GetShooterSpeed() const;
  double GetTargetRPM() const { return m_targetRPM; }
  bool IsShooting() const;
  bool IsShooterStopped() const;

  // State management
  void SetDesiredState(ShooterState state);
  void HandleStateTransition();

  void Periodic() override;

 protected:
  ShooterState m_desiredShooterState;
  ShooterState m_currShooterState;

  std::unique_ptr<MotorIO> m_shooterHoodIO;
  std::unique_ptr<MotorIO> m_shooterKickerIO;
  std::unique_ptr<MotorIO> m_shooterLeadIO;
  std::unique_ptr<MotorIO> m_shooterFollowIO;

  MotorIO::MotorIOInputs m_shooterLeadInputs;
  MotorIO::MotorIOInputs m_shooterFollowInputs;
  MotorIO::MotorIOInputs m_shooterKickerInputs;
  MotorIO::MotorIOInputs m_shooterHoodInputs;

  double m_targetRPM;
  double m_hoodAngle;

 private:
  frc::Timer m_kickerStopTimer;
  bool m_kickerStopTimerRunning = false;
  static constexpr double kKickerStopDelay = 1.0;  // seconds
};
