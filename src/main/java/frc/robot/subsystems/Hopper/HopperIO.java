package frc.robot.subsystems.Hopper;
import lombok.Getter;
import lombok.Setter;

public interface HopperIO {

    default void expandHopper(double setpoint, double rpm) {
        throw new UnsupportedOperationException("expandHopper not implemented");
    }
    default void stowHopper() {
        throw new UnsupportedOperationException("stowHopper not implemented");
    }
    /* getters */
    default void getCurrent() {
        throw new UnsupportedOperationException("getCurrent not implemented");
    }
    public class HopperIOInputs {
        // are the motors connected 
        @Getter @Setter private boolean expandingMotorConnected; 

        @Getter @Setter private boolean rollingMotorConnected; 
        // expanding motor data
        @Getter @Setter private double expandingMotorPosition; 
        @Getter @Setter private double expandingMotorVelocity; 
        @Getter @Setter private double expandingMotorVoltage; 
        @Getter @Setter private double expandingMotorTemprature;
        @Getter @Setter private double expandingMotorCurrent;
        
        // hopper roller motor data
        @Getter @Setter private double rollerMotorVelocity;
        @Getter @Setter private double rollerMotorVoltage; 
        @Getter @Setter private double rollerMotorCurrent; 
        @Getter @Setter private double rollerMotorTemprature;
    }
    default void updateInputs(HopperIOInputs hopperInputs) {}
}
