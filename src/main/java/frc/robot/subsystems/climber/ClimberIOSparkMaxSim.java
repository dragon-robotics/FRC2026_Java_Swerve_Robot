package frc.robot.subsystems.climber;

import java.util.Optional;

import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;

public class ClimberIOSparkMaxSim extends ClimberIOSparkMax {
    private final SparkSim motorSim; 
    private final DCMotor motorType; 

    public ClimberIOSparkMaxSim(int canID, SparkMaxConfig config, String motorType) {
        this(canID, config, motorType, Optional.empty());
    }

    public ClimberIOSparkMaxSim(
        int canID,
        SparkMaxConfig config,
        String motorType,
        Optional<AbsoluteEncoderConfig> absEncoderConfig) {
        super(canID, config, motorType);
        
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
    
}
