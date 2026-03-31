#pragma once

#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/intake/IntakeSubsystem.h"

class IntakeSubsystemSim : public IntakeSubsystem {
 public:
  IntakeSubsystemSim(std::unique_ptr<MotorIO> intakeRollerIO,
                     std::unique_ptr<MotorIO> intakeArmIO);

  void SimulationPeriodic() override;
  void Periodic() override;

 private:
  // Sim visualization
  frc::Mechanism2d m_mech{400, 400};
  frc::MechanismLigament2d* m_armMech = nullptr;

  // Arm sim
  frc::sim::SingleJointedArmSim m_intakeArmSim;

  static constexpr double kVisualScaleFactor = 200.0;
};
