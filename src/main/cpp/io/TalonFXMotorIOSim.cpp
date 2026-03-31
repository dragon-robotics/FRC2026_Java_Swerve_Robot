#include "io/TalonFXMotorIOSim.h"

using namespace ctre::phoenix6;

TalonFXMotorIOSim::TalonFXMotorIOSim(
    int canID, configs::TalonFXConfiguration config,
    const std::string& motorType, std::string motorName)
    : TalonFXMotorIOSim(canID, std::move(config), motorType, std::move(motorName),
                        std::nullopt, std::nullopt) {}

TalonFXMotorIOSim::TalonFXMotorIOSim(
    int canID, configs::TalonFXConfiguration config,
    const std::string& motorType, std::string motorName,
    controls::Follower followerConfig)
    : TalonFXMotorIOSim(canID, std::move(config), motorType, std::move(motorName),
                        std::nullopt, std::make_optional(std::move(followerConfig))) {}

TalonFXMotorIOSim::TalonFXMotorIOSim(
    int canID, configs::TalonFXConfiguration config,
    const std::string& motorType, std::string motorName,
    configs::CANcoderConfiguration canCoderConfig)
    : TalonFXMotorIOSim(canID, std::move(config), motorType, std::move(motorName),
                        std::make_optional(std::move(canCoderConfig)), std::nullopt) {}

TalonFXMotorIOSim::TalonFXMotorIOSim(
    int canID, configs::TalonFXConfiguration config,
    const std::string& motorType, std::string motorName,
    std::optional<configs::CANcoderConfiguration> canCoderConfig,
    std::optional<controls::Follower> followerConfig)
    : TalonFXMotorIO(canID, std::move(config), std::move(motorName),
                     std::move(canCoderConfig), std::move(followerConfig)),
      m_motorModel{ResolveMotorType(motorType)} {}

sim::TalonFXSimState& TalonFXMotorIOSim::GetSimState() {
  return m_motor.GetSimState();
}

frc::DCMotor TalonFXMotorIOSim::ResolveMotorType(const std::string& motorType) {
  if (motorType == "KrakenX60") return frc::DCMotor::KrakenX60(1);
  if (motorType == "KrakenX60_FOC") return frc::DCMotor::KrakenX60FOC(1);
  if (motorType == "KrakenX44") return frc::DCMotor::KrakenX44(1);
  if (motorType == "KrakenX44_FOC") return frc::DCMotor::KrakenX44FOC(1);
  return frc::DCMotor::KrakenX60FOC(1);
}
