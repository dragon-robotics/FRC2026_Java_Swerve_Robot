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

    private ShooterState shooterState;
    private ShooterIOInputs shooterInputs; 
    private ShooterIO shooterIO;
    public ShooterSubsystem(ShooterIO shooterIO) {
        this.shooterIO = shooterIO; 
        this.shooterState = ShooterState.STOP;
        shooterInputs = new ShooterIOInputs(); 
    }

    public ShooterState getCurrentState() {
        return shooterState;
    }
    public void setState(ShooterState newState) {
        shooterState = newState; 
    }

    public void handleShooterState() {
        switch (shooterState) {
            case STOP:
                shooterIO.stopShooter();
                break;
            case PREPFUEL:
                shooterIO.runShooter(ShooterConstants.SHOOTER_RPM / 2);
                break;
            case SHOOT:
                shooterIO.runShooter(ShooterConstants.SHOOTER_RPM);
                break;
            default:
                break;
        }
    }
    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        handleShooterState();
    } 

}
