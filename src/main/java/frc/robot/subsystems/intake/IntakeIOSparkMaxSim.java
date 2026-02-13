package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Optional;

public class IntakeIOSparkMaxSim extends IntakeIOSparkMax {

  private final SparkSim motorSim;
  private final DCMotor motorType;

  public IntakeIOSparkMaxSim(int canID, SparkMaxConfig config, String motorType) {
    this(canID, config, motorType, Optional.empty());
  }

  public IntakeIOSparkMaxSim(
      int canID,
      SparkMaxConfig config,
      String motorType,
      Optional<AbsoluteEncoderConfig> absEncoderConfig) {
    super(canID, config, motorType, absEncoderConfig);

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
