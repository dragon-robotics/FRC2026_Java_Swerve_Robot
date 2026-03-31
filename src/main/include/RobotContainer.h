#pragma once

#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/Superstructure.h"
#include "subsystems/hopper/HopperSubsystem.h"
#include "subsystems/intake/IntakeSubsystem.h"
#include "subsystems/shooter/ShooterSubsystem.h"
#include "subsystems/vision/VisionSubsystem.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  // Public subsystem references
  CommandSwerveDrivetrain* swerveSubsystem = nullptr;
  IntakeSubsystem* intakeSubsystem = nullptr;
  HopperSubsystem* hopperSubsystem = nullptr;
  ShooterSubsystem* shooterSubsystem = nullptr;
  vision::VisionSubsystem* visionSubsystem = nullptr;
  Superstructure* superstructureSubsystem = nullptr;

 private:
  void ConfigureBindings();

  frc2::CommandXboxController m_driverController;
  frc2::CommandXboxController m_operatorController;

  // Subsystem storage
  std::unique_ptr<CommandSwerveDrivetrain> m_swerveOwner;
  std::unique_ptr<IntakeSubsystem> m_intakeOwner;
  std::unique_ptr<HopperSubsystem> m_hopperOwner;
  std::unique_ptr<ShooterSubsystem> m_shooterOwner;
  std::unique_ptr<vision::VisionSubsystem> m_visionOwner;
  std::unique_ptr<Superstructure> m_superstructureOwner;

  frc::SendableChooser<frc2::Command*> m_autoChooser;
};
