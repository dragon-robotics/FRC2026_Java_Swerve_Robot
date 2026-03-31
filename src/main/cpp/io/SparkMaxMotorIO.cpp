#include "io/SparkMaxMotorIO.h"

using namespace rev::spark;

SparkMaxMotorIO::SparkMaxMotorIO(
    int canID, config::SparkMaxConfig config,
    const std::string& motorType, std::string motorName)
    : SparkMaxMotorIO(canID, std::move(config), motorType, std::move(motorName),
                      EncoderMode::PRIMARY, std::nullopt, std::nullopt) {}

SparkMaxMotorIO::SparkMaxMotorIO(
    int canID, config::SparkMaxConfig config,
    const std::string& motorType, std::string motorName,
    config::AbsoluteEncoderConfig absEncoderConfig)
    : SparkMaxMotorIO(canID, std::move(config), motorType, std::move(motorName),
                      EncoderMode::ABSOLUTE, std::make_optional(std::move(absEncoderConfig)),
                      std::nullopt) {}

SparkMaxMotorIO::SparkMaxMotorIO(
    int canID, config::SparkMaxConfig config,
    const std::string& motorType, std::string motorName,
    config::AlternateEncoderConfig altEncoderConfig)
    : SparkMaxMotorIO(canID, std::move(config), motorType, std::move(motorName),
                      EncoderMode::ALTERNATE, std::nullopt,
                      std::make_optional(std::move(altEncoderConfig))) {}

SparkMaxMotorIO::SparkMaxMotorIO(
    int canID, config::SparkMaxConfig config,
    const std::string& motorType, std::string motorName,
    EncoderMode encoderMode,
    std::optional<config::AbsoluteEncoderConfig> absEncoderConfig,
    std::optional<config::AlternateEncoderConfig> altEncoderConfig)
    : m_motor{canID, SparkMax::MotorType::kBrushless},
      m_canID{canID},
      m_config{std::move(config)},
      m_motorType{motorType},
      m_encoderMode{encoderMode},
      m_motorController{m_motor.GetClosedLoopController()} {
  m_motor.ClearFaults();

  if (absEncoderConfig.has_value()) {
    m_config.Apply(absEncoderConfig.value());
  }
  if (altEncoderConfig.has_value()) {
    m_config.Apply(altEncoderConfig.value());
  }

  m_motor.Configure(m_config, SparkBase::ResetMode::kNoResetSafeParameters,
                    SparkBase::PersistMode::kPersistParameters);
  m_motor.GetEncoder().SetPosition(0);
}

void SparkMaxMotorIO::SetMotorVoltage(double voltage) {
  m_motor.SetVoltage(units::voltage::volt_t{voltage});
}

void SparkMaxMotorIO::SetMotorPercentage(double percentage) {
  m_motor.Set(percentage);
}

void SparkMaxMotorIO::SetMotorRPM(double rpm) {
  m_motorController.SetReference(rpm, SparkBase::ControlType::kMAXMotionVelocityControl);
}

void SparkMaxMotorIO::SetMotorPosition(double setpoint, int slotID) {
  m_motorController.SetReference(
      setpoint, SparkBase::ControlType::kMAXMotionPositionControl,
      static_cast<ClosedLoopSlot>(slotID));
}

void SparkMaxMotorIO::UpdateInputs(MotorIOInputs& inputs) {
  inputs.motorConnected = (m_motor.GetDeviceId() == m_canID);
  inputs.motorVoltage = m_motor.GetAppliedOutput() * m_motor.GetBusVoltage();
  inputs.motorCurrent = m_motor.GetOutputCurrent();
  inputs.motorTemperature = m_motor.GetMotorTemperature();
  inputs.motorVelocity = m_motor.GetEncoder().GetVelocity();

  switch (m_encoderMode) {
    case EncoderMode::ABSOLUTE:
      inputs.motorPosition = m_motor.GetAbsoluteEncoder().GetPosition();
      break;
    case EncoderMode::ALTERNATE:
      inputs.motorPosition = m_motor.GetAlternateEncoder().GetPosition();
      break;
    case EncoderMode::PRIMARY:
    default:
      inputs.motorPosition = m_motor.GetEncoder().GetPosition();
      break;
  }
}
