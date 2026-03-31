#include "io/SignalRegistry.h"

#include <ctre/phoenix6/BaseStatusSignal.hpp>

SignalRegistry& SignalRegistry::GetInstance() {
  static SignalRegistry instance;
  return instance;
}

void SignalRegistry::RegisterMotorIO(MotorIO& motorIO) {
  auto signals = motorIO.GetStatusSignals();
  if (signals.empty()) {
    return;
  }
  for (auto* signal : signals) {
    m_allSignals.push_back(signal);
  }
}

void SignalRegistry::RefreshAll() {
  if (!m_allSignals.empty()) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(m_allSignals);
  }
}
