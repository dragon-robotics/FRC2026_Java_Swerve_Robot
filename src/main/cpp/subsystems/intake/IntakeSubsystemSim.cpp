#include "subsystems/intake/IntakeSubsystemSim.h"

#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/math.h>

#include "Constants.h"
#include "io/TalonFXMotorIOSim.h"
#include "io/SparkMaxMotorIOSim.h"

using namespace IntakeConstants;

IntakeSubsystemSim::IntakeSubsystemSim(
    std::unique_ptr<MotorIO> intakeRollerIO,
    std::unique_ptr<MotorIO> intakeArmIO)
    : IntakeSubsystem{std::move(intakeRollerIO), std::move(intakeArmIO)},
      m_intakeArmSim{
          frc::DCMotor::KrakenX60(1),
          kIntakeArmGearRatio,
          frc::sim::SingleJointedArmSim::EstimateMOI(
              units::meter_t{kIntakeArmLengthMeters},
              units::kilogram_t{kIntakeArmMassKg}),
          units::meter_t{kIntakeArmLengthMeters},
          units::radian_t{kIntakeMinAngleRadians},
          units::radian_t{kIntakeMaxAngleRadians},
          true,
          units::radian_t{kIntakeStartingAngleRadians}} {
  // Build mechanism visualization
  auto* root = m_mech.GetRoot("ArmRoot", 200, 200);
  auto* armBase = root->Append<frc::MechanismLigament2d>(
      "Base", 40.0, 0_deg, 20.0, frc::Color8Bit{frc::Color::kDarkGray});
  auto* tower = armBase->Append<frc::MechanismLigament2d>(
      "Tower", 30.0, 90_deg, 10.0, frc::Color8Bit{frc::Color::kGray});
  auto* pivot = tower->Append<frc::MechanismLigament2d>(
      "Pivot", 5.0, 0_deg, 5.0, frc::Color8Bit{frc::Color::kBlack});
  m_armMech = pivot->Append<frc::MechanismLigament2d>(
      "Arm", 1.0 * kVisualScaleFactor, 0_deg, 10.0, frc::Color8Bit{frc::Color::kBlue});

  frc::SmartDashboard::PutData("Intake Arm Sim", &m_mech);
}

void IntakeSubsystemSim::SimulationPeriodic() {
  // Set input voltage from motor controller
  auto* talonIO = dynamic_cast<TalonFXMotorIOSim*>(m_intakeArmIO.get());
  if (talonIO) {
    m_intakeArmSim.SetInput(
        frc::Vectord<1>{talonIO->GetSimState().GetMotorVoltage().value()});
  } else {
    m_intakeArmSim.SetInput(
        frc::Vectord<1>{m_intakeArmInputs.motorVoltage});
  }

  // Update simulation
  m_intakeArmSim.Update(20_ms);
  frc::sim::RoboRioSim::SetVInVoltage(
      frc::sim::BatterySim::Calculate({m_intakeArmSim.GetCurrentDraw()}));

  double motorPositionRotations =
      units::turn_t{m_intakeArmSim.GetAngle() * kIntakeArmGearRatio}.value();
  double motorVelocityRPS =
      units::turns_per_second_t{m_intakeArmSim.GetVelocity() * kIntakeArmGearRatio}.value();

  if (talonIO) {
    talonIO->GetSimState().SetRawRotorPosition(units::turn_t{motorPositionRotations});
    talonIO->GetSimState().SetRotorVelocity(
        units::turns_per_second_t{motorVelocityRPS});
  }
}

void IntakeSubsystemSim::Periodic() {
  // Update arm visualization angle
  double currentAngleRad = m_intakeArmSim.GetAngle().value();
  if (m_armMech) {
    m_armMech->SetAngle(units::degree_t{units::radian_t{currentAngleRad}});
  }

  HandleStateTransition();

  m_intakeRollerIO->UpdateInputs(m_intakeRollerInputs);
  m_intakeArmIO->UpdateInputs(m_intakeArmInputs);
}
