package frc.robot.subsystems.Shooter;

import java.lang.constant.Constable;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;
import frc.robot.Constants;
import frc.robot.Constants.*;
public class ShooterSubsystem extends SubsystemBase{
    public enum ShooterState {
        STOP, 
        PREPFUEL,
        SHOOT
    }

    private ShooterState desiredShooterState;
    private ShooterState currentShooterState; 

    private ShooterIOInputs shooterInputs; 
    private ShooterIO shooterIO;
    public ShooterSubsystem(ShooterIO shooterIO) {
        this.shooterIO = shooterIO; 
        this.shooterInputs = new ShooterIOInputs();

        // initialize shooter states 
        this.desiredShooterState = ShooterState.STOP;
        this.currentShooterState = ShooterState.STOP;
    }

    /* Setters */
    public void setDesiredState(ShooterState state) {
        this.desiredShooterState = state; 
        switch (desiredShooterState) {
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
        return getShooterSpeed() == 0; 
    }

    @Override
    public void periodic() {
        switch (currentShooterState) {
            case STOP:
                stopShooter();
                break;
            case PREPFUEL: 
                runShooter();
                break;
            case SHOOT: 
                runShooter();
            default:
                break;
        }
    } 

}
