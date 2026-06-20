package frc.robot.io;

import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Optional;

/** Simulation-backed Spark MAX IO wrapper. */
public class SparkMaxMotorIOSim extends SparkMaxMotorIO {
  private static final String NEO = "NEO";
  private static final String NEO550 = "NEO550";
  private static final String VORTEX = "Vortex";

  private final SparkSim motorSim;
  private final DCMotor motorType;

  /** Creates a simulated Spark MAX using the primary relative encoder. */
  public SparkMaxMotorIOSim(int canID, SparkMaxConfig config, String motorType, String motorName) {
    this(
        canID,
        config,
        motorType,
        motorName,
        EncoderMode.PRIMARY,
        Optional.empty(),
        Optional.empty());
  }

  /** Creates a simulated Spark MAX using the absolute encoder. */
  public SparkMaxMotorIOSim(
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

  /** Creates a simulated Spark MAX using the alternate encoder. */
  public SparkMaxMotorIOSim(
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

  /** Full constructor for selecting the encoder source and motor model string. */
  public SparkMaxMotorIOSim(
      int canID,
      SparkMaxConfig config,
      String motorType,
      String motorName,
      EncoderMode encoderMode,
      Optional<AbsoluteEncoderConfig> absEncoderConfig,
      Optional<AlternateEncoderConfig> altEncoderConfig) {
    super(canID, config, motorType, motorName, encoderMode, absEncoderConfig, altEncoderConfig);
    this.motorType = resolveMotorType(motorType);
    this.motorSim = new SparkSim(this.motor, this.motorType);
  }

  /** Returns the REV simulation wrapper for this motor. */
  public SparkSim getMotorSim() {
    return motorSim;
  }

  /** Returns the WPILib motor model used by the simulation wrapper. */
  public DCMotor getMotorType() {
    return motorType;
  }

  private static DCMotor resolveMotorType(String motorType) {
    return switch (motorType) {
      case NEO -> DCMotor.getNEO(1);
      case NEO550 -> DCMotor.getNeo550(1);
      case VORTEX -> DCMotor.getNeoVortex(1);
      default -> DCMotor.getNEO(1);
    };
  }
}
