#include "io/SparkMaxMotorIOSim.h"

using namespace rev::spark;

SparkMaxMotorIOSim::SparkMaxMotorIOSim(
    int canID, config::SparkMaxConfig config,
    const std::string& motorType, std::string motorName)
    : SparkMaxMotorIOSim(canID, std::move(config), motorType, std::move(motorName),
                         EncoderMode::PRIMARY, std::nullopt, std::nullopt) {}

SparkMaxMotorIOSim::SparkMaxMotorIOSim(
    int canID, config::SparkMaxConfig config,
    const std::string& motorType, std::string motorName,
    config::AbsoluteEncoderConfig absEncoderConfig)
    : SparkMaxMotorIOSim(canID, std::move(config), motorType, std::move(motorName),
                         EncoderMode::ABSOLUTE,
                         std::make_optional(std::move(absEncoderConfig)), std::nullopt) {}

SparkMaxMotorIOSim::SparkMaxMotorIOSim(
    int canID, config::SparkMaxConfig config,
    const std::string& motorType, std::string motorName,
    config::AlternateEncoderConfig altEncoderConfig)
    : SparkMaxMotorIOSim(canID, std::move(config), motorType, std::move(motorName),
                         EncoderMode::ALTERNATE, std::nullopt,
                         std::make_optional(std::move(altEncoderConfig))) {}

SparkMaxMotorIOSim::SparkMaxMotorIOSim(
    int canID, config::SparkMaxConfig config,
    const std::string& motorType, std::string motorName,
    EncoderMode encoderMode,
    std::optional<config::AbsoluteEncoderConfig> absEncoderConfig,
    std::optional<config::AlternateEncoderConfig> altEncoderConfig)
    : SparkMaxMotorIO(canID, std::move(config), motorType, std::move(motorName),
                      encoderMode, std::move(absEncoderConfig), std::move(altEncoderConfig)),
      m_motorModel{[&]() -> frc::DCMotor {
        if (motorType == "NEO") return frc::DCMotor::NEO(1);
        if (motorType == "NEO550") return frc::DCMotor::Neo550(1);
        if (motorType == "Vortex") return frc::DCMotor::NeoVortex(1);
        return frc::DCMotor::NEO(1);
      }()},
      m_motorSim{m_motor, m_motorModel} {}
