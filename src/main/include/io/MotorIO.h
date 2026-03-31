#pragma once

#include <ctre/phoenix6/BaseStatusSignal.hpp>
#include <string>
#include <vector>

class MotorIO {
 public:
  struct MotorIOInputs {
    bool motorConnected = false;
    double motorPosition = 0.0;
    double motorVelocity = 0.0;
    double motorVoltage = 0.0;
    double motorCurrent = 0.0;
    double motorTemperature = 0.0;
  };

  virtual ~MotorIO() = default;

  virtual void SetMotorVoltage(double voltage) {}
  virtual void SetMotorPercentage(double percentage) {}
  virtual void SetMotorRPM(double rpm) {}
  virtual void SetMotorPosition(double setpoint) {}
  virtual void SetMotorPosition(double setpoint, int slotID) {}
  virtual void UpdateInputs(MotorIOInputs& inputs) {}

  virtual std::string GetMotorName() const { return "Unknown"; }

  virtual std::vector<ctre::phoenix6::BaseStatusSignal*> GetStatusSignals() {
    return {};
  }
};
