#pragma once

#include "io/MotorIO.h"

#include <rev/spark/SparkMax.h>
#include <rev/spark/SparkClosedLoopController.h>
#include <rev/spark/config/SparkMaxConfig.h>
#include <rev/spark/config/AbsoluteEncoderConfig.h>
#include <rev/spark/config/AlternateEncoderConfig.h>

#include <optional>
#include <string>

class SparkMaxMotorIO : public MotorIO {
 public:
  enum class EncoderMode {
    PRIMARY,
    ABSOLUTE,
    ALTERNATE
  };

  SparkMaxMotorIO(int canID, rev::spark::config::SparkMaxConfig config,
                  const std::string& motorType, std::string motorName);

  SparkMaxMotorIO(int canID, rev::spark::config::SparkMaxConfig config,
                  const std::string& motorType, std::string motorName,
                  rev::spark::config::AbsoluteEncoderConfig absEncoderConfig);

  SparkMaxMotorIO(int canID, rev::spark::config::SparkMaxConfig config,
                  const std::string& motorType, std::string motorName,
                  rev::spark::config::AlternateEncoderConfig altEncoderConfig);

  SparkMaxMotorIO(int canID, rev::spark::config::SparkMaxConfig config,
                  const std::string& motorType, std::string motorName,
                  EncoderMode encoderMode,
                  std::optional<rev::spark::config::AbsoluteEncoderConfig> absEncoderConfig,
                  std::optional<rev::spark::config::AlternateEncoderConfig> altEncoderConfig);

  void SetMotorVoltage(double voltage) override;
  void SetMotorPercentage(double percentage) override;
  void SetMotorRPM(double rpm) override;
  void SetMotorPosition(double setpoint, int slotID) override;
  void UpdateInputs(MotorIOInputs& inputs) override;

 protected:
  rev::spark::SparkMax m_motor;
  int m_canID;
  rev::spark::config::SparkMaxConfig m_config;
  std::string m_motorType;
  EncoderMode m_encoderMode;

 private:
  rev::spark::SparkClosedLoopController m_motorController;
};
