#pragma once

#include "io/TalonFXMotorIO.h"

#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/system/plant/DCMotor.h>

#include <optional>
#include <string>

class TalonFXMotorIOSim : public TalonFXMotorIO {
 public:
  TalonFXMotorIOSim(int canID, ctre::phoenix6::configs::TalonFXConfiguration config,
                    const std::string& motorType, std::string motorName);

  TalonFXMotorIOSim(int canID, ctre::phoenix6::configs::TalonFXConfiguration config,
                    const std::string& motorType, std::string motorName,
                    ctre::phoenix6::controls::Follower followerConfig);

  TalonFXMotorIOSim(int canID, ctre::phoenix6::configs::TalonFXConfiguration config,
                    const std::string& motorType, std::string motorName,
                    ctre::phoenix6::configs::CANcoderConfiguration canCoderConfig);

  TalonFXMotorIOSim(int canID, ctre::phoenix6::configs::TalonFXConfiguration config,
                    const std::string& motorType, std::string motorName,
                    std::optional<ctre::phoenix6::configs::CANcoderConfiguration> canCoderConfig,
                    std::optional<ctre::phoenix6::controls::Follower> followerConfig);

  ctre::phoenix6::sim::TalonFXSimState& GetSimState();
  frc::DCMotor GetMotorType() const { return m_motorModel; }

 private:
  static frc::DCMotor ResolveMotorType(const std::string& motorType);
  frc::DCMotor m_motorModel;
};
