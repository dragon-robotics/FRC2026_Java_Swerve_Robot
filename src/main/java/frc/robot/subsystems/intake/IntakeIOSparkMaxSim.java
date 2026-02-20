package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Optional;

public class IntakeIOSparkMaxSim extends IntakeIOSparkMax {

  private final SparkSim motorSim;
  private final DCMotor motorType;

  public IntakeIOSparkMaxSim(int canID, SparkMaxConfig config, String motorType, String motorName) {
    this(
        canID,
        config,
        motorType,
        motorName,
        EncoderMode.PRIMARY,
        Optional.empty(),
        Optional.empty());
  }

  public IntakeIOSparkMaxSim(
      int canID,
      SparkMaxConfig config,
      String motorType,
      String motorName,
      AbsoluteEncoderConfig absEncoderConfig) {
    this(
        canID,
        config,
        motorType,
        motorName,
        EncoderMode.ABSOLUTE,
        Optional.of(absEncoderConfig),
        Optional.empty());
  }

  public IntakeIOSparkMaxSim(
      int canID,
      SparkMaxConfig config,
      String motorType,
      String motorName,
      AlternateEncoderConfig altEncoderConfig) {
    this(
        canID,
        config,
        motorType,
        motorName,
        EncoderMode.ALTERNATE,
        Optional.empty(),
        Optional.of(altEncoderConfig));
  }

  public IntakeIOSparkMaxSim(
      int canID,
      SparkMaxConfig config,
      String motorType,
      String motorName,
      EncoderMode encoderMode,
      Optional<AbsoluteEncoderConfig> absEncoderConfig,
      Optional<AlternateEncoderConfig> altEncoderConfig) {
    super(canID, config, motorType, motorName, encoderMode, absEncoderConfig, altEncoderConfig);
    // Initialize SparkSim with appropriate parameters for the motor type
    switch (motorType) {
      case "NEO":
        this.motorType = DCMotor.getNEO(1);
        break;
      case "NEO550":
        this.motorType = DCMotor.getNeo550(1);
        break;
      case "Vortex":
        this.motorType = DCMotor.getNeoVortex(1);
        break;
      default:
        this.motorType = DCMotor.getNEO(1);
    }

    this.motorSim = new SparkSim(this.motor, this.motorType);
  }

  public SparkSim getMotorSim() {
    return motorSim;
  }

  public DCMotor getMotorType() {
    return motorType;
  }
}
