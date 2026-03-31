#pragma once

#include <frc2/command/SubsystemBase.h>

#include "io/MotorIO.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  enum class IntakeState {
    HOME,
    INTAKE,
    OUTTAKE,
    DEPLOYED,
    DEPLOYING,
    STOWING,
    WOKTOSS,
    WOKTOSSING,
    AUTO_WOKTOSSING,
    JUICER
  };

  enum class JuicerPhase {
    PRE_JUICE,
    SQUEEZE
  };

  IntakeSubsystem(std::unique_ptr<MotorIO> intakeRollerIO,
                  std::unique_ptr<MotorIO> intakeArmIO);

  // Motor control
  void RunIntake();
  void RunOuttake();
  void StopIntake();
  void DeployIntakeArm();
  void CoastIntakeArm();
  void StowIntakeArm();
  void WokTossIntakeArm();
  void SetIntakeArmSetpoint(double setpoint, int slotID);

  // Getters
  IntakeState GetCurrentState() const { return m_currIntakeState; }
  IntakeState GetDesiredState() const { return m_desiredIntakeState; }
  double GetIntakeArmSetpoint() const { return m_intakeArmInputs.motorPosition; }
  double GetIntakeRollerSpeed() const { return m_intakeRollerInputs.motorVelocity; }
  bool IsIntakeArmAtDeployed() const;
  bool IsIntakeArmAtStowed() const;
  bool IsIntakeArmAtWokToss() const;
  bool IsIntakeArmAtPreJuice() const;
  bool IsIntaking() const { return GetIntakeRollerSpeed() > 5; }
  bool IsOuttaking() const { return GetIntakeRollerSpeed() < -5; }

  // State management
  void SetDesiredState(IntakeState state);
  void HandleStateTransition();

  void Periodic() override;

 protected:
  IntakeState m_currIntakeState;
  IntakeState m_desiredIntakeState;

  std::unique_ptr<MotorIO> m_intakeRollerIO;
  std::unique_ptr<MotorIO> m_intakeArmIO;
  MotorIO::MotorIOInputs m_intakeRollerInputs;
  MotorIO::MotorIOInputs m_intakeArmInputs;

  bool m_wokTossMovingToDeployed = false;

 private:
  IntakeState m_lastCommandedState;
  bool m_lastCommandedStateValid = false;
  bool m_homeCoasting = false;

  JuicerPhase m_juicerPhase = JuicerPhase::PRE_JUICE;
  JuicerPhase m_lastJuicerPhase;
  bool m_lastJuicerPhaseValid = false;
};
