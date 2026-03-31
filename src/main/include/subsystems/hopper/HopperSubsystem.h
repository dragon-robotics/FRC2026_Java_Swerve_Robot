#pragma once

#include <frc2/command/SubsystemBase.h>

#include "io/MotorIO.h"

class HopperSubsystem : public frc2::SubsystemBase {
 public:
  enum class HopperState {
    STOP,
    INDEXTOSHOOTER,
    INDEXTOINTAKE
  };

  HopperSubsystem(std::unique_ptr<MotorIO> leadRollerMotorIO,
                  std::unique_ptr<MotorIO> followRollerMotorIO);

  // Motor control
  void RunHopperRollerRPM(double rpm);
  void RunHopperRollerVoltage(double voltage);
  void RunHopperRollerPercentage(double percentage);
  void IndexToShooter();
  void IndexToIntake();
  void StopHopperRoller();

  // Getters
  HopperState GetCurrentState() const { return m_currHopperState; }
  HopperState GetDesiredState() const { return m_desiredHopperState; }

  // State management
  void SetDesiredState(HopperState state);
  void HandleStateTransition();

  void Periodic() override;

 private:
  HopperState m_currHopperState;
  HopperState m_desiredHopperState;
  HopperState m_lastCommandedState;
  bool m_lastCommandedStateValid = false;

  std::unique_ptr<MotorIO> m_leadRollerMotorIO;
  std::unique_ptr<MotorIO> m_followRollerMotorIO;
  MotorIO::MotorIOInputs m_leadRollerMotorIOInputs;
  MotorIO::MotorIOInputs m_followRollerMotorIOInputs;
};
