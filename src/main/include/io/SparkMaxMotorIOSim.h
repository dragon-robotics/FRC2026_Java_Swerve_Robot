#pragma once

#include "io/SparkMaxMotorIO.h"

#include <rev/spark/SparkSim.h>
#include <frc/system/plant/DCMotor.h>

class SparkMaxMotorIOSim : public SparkMaxMotorIO {
 public:
  SparkMaxMotorIOSim(int canID, rev::spark::config::SparkMaxConfig config,
                     const std::string& motorType, std::string motorName);

  SparkMaxMotorIOSim(int canID, rev::spark::config::SparkMaxConfig config,
                     const std::string& motorType, std::string motorName,
                     rev::spark::config::AbsoluteEncoderConfig absEncoderConfig);

  SparkMaxMotorIOSim(int canID, rev::spark::config::SparkMaxConfig config,
                     const std::string& motorType, std::string motorName,
                     rev::spark::config::AlternateEncoderConfig altEncoderConfig);

  SparkMaxMotorIOSim(int canID, rev::spark::config::SparkMaxConfig config,
                     const std::string& motorType, std::string motorName,
                     EncoderMode encoderMode,
                     std::optional<rev::spark::config::AbsoluteEncoderConfig> absEncoderConfig,
                     std::optional<rev::spark::config::AlternateEncoderConfig> altEncoderConfig);

  rev::spark::SparkSim& GetMotorSim() { return m_motorSim; }
  frc::DCMotor GetMotorType() const { return m_motorModel; }

 private:
  frc::DCMotor m_motorModel;
  rev::spark::SparkSim m_motorSim;
};
