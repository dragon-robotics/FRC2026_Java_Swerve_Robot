#pragma once

#include "io/MotorIO.h"

#include <ctre/phoenix6/BaseStatusSignal.hpp>
#include <vector>

class SignalRegistry {
 public:
  static SignalRegistry& GetInstance();

  void RegisterMotorIO(MotorIO& motorIO);
  void RefreshAll();

 private:
  SignalRegistry() = default;
  SignalRegistry(const SignalRegistry&) = delete;
  SignalRegistry& operator=(const SignalRegistry&) = delete;

  std::vector<ctre::phoenix6::BaseStatusSignal*> m_allSignals;
};
