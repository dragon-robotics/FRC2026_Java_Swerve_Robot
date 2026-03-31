#include "io/TalonFXMotorIOTunable.h"
#include "util/TalonFXTunableHelper.h"

using namespace ctre::phoenix6;

TalonFXMotorIOTunable::TalonFXMotorIOTunable(
    int canID, configs::TalonFXConfiguration config, std::string motorName)
    : TalonFXMotorIOTunable(canID, std::move(config), std::move(motorName),
                            std::nullopt, std::nullopt) {}

TalonFXMotorIOTunable::TalonFXMotorIOTunable(
    int canID, configs::TalonFXConfiguration config, std::string motorName,
    controls::Follower followerConfig)
    : TalonFXMotorIOTunable(canID, std::move(config), std::move(motorName),
                            std::nullopt, std::make_optional(std::move(followerConfig))) {}

TalonFXMotorIOTunable::TalonFXMotorIOTunable(
    int canID, configs::TalonFXConfiguration config, std::string motorName,
    configs::CANcoderConfiguration canCoderConfig)
    : TalonFXMotorIOTunable(canID, std::move(config), std::move(motorName),
                            std::make_optional(std::move(canCoderConfig)), std::nullopt) {}

TalonFXMotorIOTunable::TalonFXMotorIOTunable(
    int canID, configs::TalonFXConfiguration config, std::string motorName,
    std::optional<configs::CANcoderConfiguration> canCoderConfig,
    std::optional<controls::Follower> followerConfig)
    : TalonFXMotorIO(canID, std::move(config), std::move(motorName),
                     std::move(canCoderConfig), std::move(followerConfig)) {
  std::string subsystem = ExtractSubsystem(m_motorName);
  std::string baseKey = subsystem + "/Tuning/TalonFX/" + m_motorName + "/";
  m_tunableHelper = std::make_unique<TalonFXTunableHelper>(
      m_motor, baseKey, m_config, m_motorName);
}

std::string TalonFXMotorIOTunable::ExtractSubsystem(const std::string& motorName) {
  auto space = motorName.find(' ');
  return (space != std::string::npos) ? motorName.substr(0, space) : motorName;
}
