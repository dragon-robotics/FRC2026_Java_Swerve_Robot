package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Optional;

public class ShooterIOSparkMaxSim extends ShooterIOSparkMax {

  private final SparkSim motorSim;
  private final DCMotor motorType;

  public ShooterIOSparkMaxSim(int canID, SparkMaxConfig config, String motorType) {
    this(canID, config, motorType, Optional.empty());
  }

  public ShooterIOSparkMaxSim(
      int canID,
      SparkMaxConfig config,
      String motorType,
      Optional<AlternateEncoderConfig> altEncoderConfig) {
    super(canID, config, motorType, altEncoderConfig);
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
