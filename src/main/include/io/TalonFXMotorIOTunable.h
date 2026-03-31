#pragma once

#include "io/TalonFXMotorIO.h"

#include <optional>
#include <string>
#include <memory>

class TalonFXTunableHelper;

class TalonFXMotorIOTunable : public TalonFXMotorIO {
 public:
  TalonFXMotorIOTunable(int canID, ctre::phoenix6::configs::TalonFXConfiguration config,
                        std::string motorName);

  TalonFXMotorIOTunable(int canID, ctre::phoenix6::configs::TalonFXConfiguration config,
                        std::string motorName,
                        ctre::phoenix6::controls::Follower followerConfig);

  TalonFXMotorIOTunable(int canID, ctre::phoenix6::configs::TalonFXConfiguration config,
                        std::string motorName,
                        ctre::phoenix6::configs::CANcoderConfiguration canCoderConfig);

  TalonFXMotorIOTunable(int canID, ctre::phoenix6::configs::TalonFXConfiguration config,
                        std::string motorName,
                        std::optional<ctre::phoenix6::configs::CANcoderConfiguration> canCoderConfig,
                        std::optional<ctre::phoenix6::controls::Follower> followerConfig);

 private:
  static std::string ExtractSubsystem(const std::string& motorName);
  std::unique_ptr<TalonFXTunableHelper> m_tunableHelper;
};
