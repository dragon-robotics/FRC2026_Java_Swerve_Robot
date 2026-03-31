#include "io/TalonFXMotorIO.h"
#include "Constants.h"

#include <ctre/phoenix6/BaseStatusSignal.hpp>

using namespace ctre::phoenix6;

TalonFXMotorIO::TalonFXMotorIO(int canID,
                                 configs::TalonFXConfiguration config,
                                 std::string motorName)
    : TalonFXMotorIO(canID, std::move(config), std::move(motorName),
                     std::nullopt, std::nullopt) {}

TalonFXMotorIO::TalonFXMotorIO(int canID,
                                 configs::TalonFXConfiguration config,
                                 std::string motorName,
                                 controls::Follower followerConfig)
    : TalonFXMotorIO(canID, std::move(config), std::move(motorName),
                     std::nullopt, std::make_optional(std::move(followerConfig))) {}

TalonFXMotorIO::TalonFXMotorIO(int canID,
                                 configs::TalonFXConfiguration config,
                                 std::string motorName,
                                 configs::CANcoderConfiguration canCoderConfig)
    : TalonFXMotorIO(canID, std::move(config), std::move(motorName),
                     std::make_optional(std::move(canCoderConfig)), std::nullopt) {}

TalonFXMotorIO::TalonFXMotorIO(
    int canID,
    configs::TalonFXConfiguration config,
    std::string motorName,
    std::optional<configs::CANcoderConfiguration> canCoderConfig,
    std::optional<controls::Follower> followerConfig)
    : m_motor{canID},
      m_canID{canID},
      m_config{std::move(config)},
      m_motorName{std::move(motorName)},
      m_motorVoltageSignal{m_motor.GetMotorVoltage()},
      m_statorCurrentSignal{m_motor.GetStatorCurrent()},
      m_deviceTempSignal{m_motor.GetDeviceTemp()},
      m_velocitySignal{m_motor.GetVelocity()},
      m_positionSignal{m_motor.GetPosition()},
      m_dutyCycleSignal{m_motor.GetDutyCycle()},
      m_torqueCurrentSignal{m_motor.GetTorqueCurrent()} {
  m_motor.ClearStickyFaults();

  // Disable all unused signals first
  m_motor.OptimizeBusUtilization(0_Hz);

  // Re-enable update for signals that we are using
  BaseStatusSignal::SetUpdateFrequencyForAll(
      100_Hz,
      m_dutyCycleSignal,
      m_motorVoltageSignal,
      m_torqueCurrentSignal,
      m_statorCurrentSignal,
      m_velocitySignal,
      m_positionSignal);

  m_deviceTempSignal.SetUpdateFrequency(0.25_Hz);

  // Register the motor to the signal registry
  SignalRegistry::GetInstance().RegisterMotorIO(*this);

  // Apply CANcoder config if present
  if (canCoderConfig.has_value()) {
    m_canCoder = std::make_unique<hardware::CANcoder>(canID);
    m_canCoder->GetConfigurator().Apply(canCoderConfig.value());

    configs::TalonFXConfiguration updatedConfig = m_config;
    updatedConfig.Feedback.FeedbackRemoteSensorID = m_canCoder->GetDeviceID();
    updatedConfig.Feedback.FeedbackSensorSource =
        signals::FeedbackSensorSourceValue::FusedCANcoder;
    m_motor.GetConfigurator().Apply(updatedConfig);

    m_canCoder->OptimizeBusUtilization(0_Hz);

    BaseStatusSignal::SetUpdateFrequencyForAll(
        100_Hz,
        m_canCoder->GetPosition(),
        m_canCoder->GetVelocity(),
        m_canCoder->GetAbsolutePosition());
  } else {
    m_motor.GetConfigurator().Apply(m_config);
  }

  // Apply follower config if present
  if (followerConfig.has_value()) {
    m_motor.SetControl(followerConfig.value());
  }
}

std::string TalonFXMotorIO::GetMotorName() const {
  return m_motorName;
}

void TalonFXMotorIO::SetMotorVoltage(double voltage) {
  m_motor.SetVoltage(units::voltage::volt_t{voltage});
}

void TalonFXMotorIO::SetMotorPercentage(double percentage) {
  m_motor.Set(percentage);
}

void TalonFXMotorIO::SetMotorRPM(double rpm) {
  m_motor.SetControl(m_velocityRequest.WithVelocity(
      units::angular_velocity::turns_per_second_t{rpm / 60.0}));
}

void TalonFXMotorIO::SetMotorPosition(double setpoint) {
  m_motor.SetControl(m_positionVoltageRequest.WithPosition(
      units::angle::turn_t{setpoint}));
}

void TalonFXMotorIO::SetMotorPosition(double setpoint, int slotID) {
  m_motor.SetControl(m_positionVoltageRequest
      .WithPosition(units::angle::turn_t{setpoint})
      .WithSlot(slotID));
}

std::vector<BaseStatusSignal*> TalonFXMotorIO::GetStatusSignals() {
  return {
    &m_motorVoltageSignal,
    &m_statorCurrentSignal,
    &m_deviceTempSignal,
    &m_velocitySignal,
    &m_positionSignal,
    &m_dutyCycleSignal,
    &m_torqueCurrentSignal
  };
}

void TalonFXMotorIO::UpdateInputs(MotorIOInputs& inputs) {
  inputs.motorConnected = BaseStatusSignal::IsAllGood(
      m_motorVoltageSignal, m_statorCurrentSignal,
      m_velocitySignal, m_positionSignal);
  inputs.motorVoltage = m_motorVoltageSignal.GetValueAsDouble();
  inputs.motorCurrent = m_statorCurrentSignal.GetValueAsDouble();
  inputs.motorTemperature = m_deviceTempSignal.GetValueAsDouble();
  inputs.motorVelocity = m_velocitySignal.GetValueAsDouble();
  inputs.motorPosition = m_positionSignal.GetValueAsDouble();
}
