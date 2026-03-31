#pragma once

#include "io/MotorIO.h"
#include "io/SignalRegistry.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/MotionMagicVelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/MotionMagicExpoTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>

#include <optional>
#include <string>
#include <memory>
#include <vector>

class TalonFXMotorIO : public MotorIO {
 public:
  TalonFXMotorIO(int canID, ctre::phoenix6::configs::TalonFXConfiguration config,
                 std::string motorName);

  TalonFXMotorIO(int canID, ctre::phoenix6::configs::TalonFXConfiguration config,
                 std::string motorName, ctre::phoenix6::controls::Follower followerConfig);

  TalonFXMotorIO(int canID, ctre::phoenix6::configs::TalonFXConfiguration config,
                 std::string motorName,
                 ctre::phoenix6::configs::CANcoderConfiguration canCoderConfig);

  TalonFXMotorIO(int canID, ctre::phoenix6::configs::TalonFXConfiguration config,
                 std::string motorName,
                 std::optional<ctre::phoenix6::configs::CANcoderConfiguration> canCoderConfig,
                 std::optional<ctre::phoenix6::controls::Follower> followerConfig);

  void SetMotorVoltage(double voltage) override;
  void SetMotorPercentage(double percentage) override;
  void SetMotorRPM(double rpm) override;
  void SetMotorPosition(double setpoint) override;
  void SetMotorPosition(double setpoint, int slotID) override;
  void UpdateInputs(MotorIOInputs& inputs) override;
  std::string GetMotorName() const override;
  std::vector<ctre::phoenix6::BaseStatusSignal*> GetStatusSignals() override;

  ctre::phoenix6::hardware::TalonFX& GetMotor() { return m_motor; }

 protected:
  ctre::phoenix6::hardware::TalonFX m_motor;
  int m_canID;
  ctre::phoenix6::configs::TalonFXConfiguration m_config;
  std::string m_motorName;
  std::unique_ptr<ctre::phoenix6::hardware::CANcoder> m_canCoder;

  ctre::phoenix6::controls::VelocityTorqueCurrentFOC m_velocityRequest{0_tps};
  ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC m_mmVelocityRequest{0_tps};
  ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC m_mmPositionRequest{0_tr};
  ctre::phoenix6::controls::PositionVoltage m_positionVoltageRequest{0_tr};

  ctre::phoenix6::StatusSignal<units::voltage::volt_t>& m_motorVoltageSignal;
  ctre::phoenix6::StatusSignal<units::current::ampere_t>& m_statorCurrentSignal;
  ctre::phoenix6::StatusSignal<units::temperature::celsius_t>& m_deviceTempSignal;
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t>& m_velocitySignal;
  ctre::phoenix6::StatusSignal<units::angle::turn_t>& m_positionSignal;
  ctre::phoenix6::StatusSignal<units::dimensionless::scalar_t>& m_dutyCycleSignal;
  ctre::phoenix6::StatusSignal<units::current::ampere_t>& m_torqueCurrentSignal;
};
