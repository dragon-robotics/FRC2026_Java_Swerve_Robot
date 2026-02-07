package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
public class ShooterSubsystem extends SubsystemBase{
    public enum ShooterState {
        STOP, 
        PREPFUEL,
        SHOOT
    }

    private ShooterState desiredShooterState;
    private ShooterState currentShooterState; 

    private ShooterIO shooterIO;
    private ShooterIOInputs shooterInputs;
    
    public ShooterSubsystem(ShooterIO shooterIO) {
        this.shooterIO = shooterIO; 
        this.shooterInputs = new ShooterIOInputs();

        // initialize shooter states 
        this.desiredShooterState = ShooterState.STOP;
        this.currentShooterState = ShooterState.STOP;
    }

    /* Setters */
    public void setDesiredState(ShooterState state) {
        switch (state) {
            case STOP:
                currentShooterState = ShooterState.STOP;
                break;
            case PREPFUEL:
                currentShooterState = ShooterState.PREPFUEL;
                break;
            case SHOOT:
                currentShooterState = ShooterState.SHOOT; 
                break;
            default:
                break;
        }
    }
    
    public ShooterState getCurrentState() {
        return this.currentShooterState; 
    }

    public ShooterState getShooterState() {
        return this.desiredShooterState; 
    }

    /* Getters */
    public void runShooter() {
        shooterIO.runShooter(ShooterConstants.SHOOTER_RPM);
    }
    
    // runs the shooter at half speed 
    public void prepShooter() {
        shooterIO.runShooter(ShooterConstants.SHOOTER_RPM * 0.5);
    }

    public void stopShooter() {
        shooterIO.stopShooter();
    }

    public double getShooterSpeed() {
        return shooterIO.getShooterSpeed(); 
    }
    
    public boolean isShooting() {
        return getShooterSpeed() > 5; 
    }

    public boolean isShooterStopped() {
        return Math.abs(getShooterSpeed()) < 1.0; // Use threshold instead of exact comparison
    }

    @Override
    public void periodic() {
        switch (currentShooterState) {
            case STOP:
                stopShooter();
                break;
            case PREPFUEL: 
                prepShooter();
                break;
            case SHOOT: 
                runShooter();
                break;
            default:
                break;
        }
        shooterIO.updateInputs(shooterInputs);
    } 
}
