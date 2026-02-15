package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterSubsystemConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class ShooterSubsystem extends SubsystemBase {
  public enum ShooterState {
    STOP,
    PREPFUEL,
    SHOOT,
    TRANSITION  // Transition state to handle ramping up/down of the shooter speed
  }

  private ShooterState desiredShooterState;
  private ShooterState currShooterState;

  private ShooterIO shooterLeadIO, shooterFollowIO;
  private ShooterIOInputs shooterLeadInputs, shooterFollowInputs;

  private double targetRPM;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(ShooterIO shooterLeadIO, ShooterIO shooterFollowIO) {
    this.shooterLeadIO = shooterLeadIO;
    this.shooterFollowIO = shooterFollowIO;
    this.shooterLeadInputs = new ShooterIOInputs();
    this.shooterFollowInputs = new ShooterIOInputs();

    // initialize shooter states
    this.desiredShooterState = ShooterState.STOP;
    this.currShooterState = ShooterState.STOP;
  }

  /* Setters */
  public void setDesiredState(ShooterState state) {
    this.desiredShooterState = state;

    switch (desiredShooterState) {
      case STOP:
        currShooterState = ShooterState.TRANSITION;
        break;
      case PREPFUEL:
        currShooterState = ShooterState.TRANSITION;
        break;
      case SHOOT:
        currShooterState = ShooterState.TRANSITION;
        break;
      default:
        break;        
    }
  }

  public ShooterState getCurrentState() {
    return this.currShooterState;
  }

  public ShooterState getShooterState() {
    return this.desiredShooterState;
  }

  public void runShooter() {
    shooterLeadIO.setMotorRPM(targetRPM);
  }

  // runs the shooter at half speed
  public void prepShooter() {
    shooterLeadIO.setMotorRPM(targetRPM * 0.3); // Run at 30% of target RPM for prep
  }

  public void stopShooter() {
    shooterLeadIO.setMotorRPM(0);
  }

  public double getShooterSpeed() {
    return shooterLeadInputs.getMotorVelocity();
  }

  public boolean isShooting() {
    return getShooterSpeed() > 5;
  }

  public boolean isShooterStopped() {
    return Math.abs(getShooterSpeed()) < 1.0; // Use threshold instead of exact comparison
  }

  public boolean calculateTargetRPM(double distanceToTarget) {
    // Implement a method to calculate target RPM based on distance to target
    // This is a placeholder implementation and should be replaced with actual logic
    if (distanceToTarget < 1.0) {
      targetRPM = SHOOTER_RPM * 0.5; // Closer targets require less RPM
    } else {
      targetRPM = SHOOTER_RPM; // Farther targets require full RPM
    }
    return targetRPM > 0;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public void handleStateTransition() {
    // Handle the state transitions
    switch (currShooterState) {
      case STOP:
        stopShooter();
        break;
      case PREPFUEL:
        prepShooter();
        break;
      case SHOOT:
        runShooter();
        break;
      case TRANSITION:
        switch (desiredShooterState) {
          case STOP:
            stopShooter();
            if(isShooterStopped()) {
              currShooterState = ShooterState.STOP;
            }
            break;
          case PREPFUEL:
            prepShooter();
            if (MathUtil.isNear(targetRPM * 0.3, getShooterSpeed(), 5)) {
              currShooterState = ShooterState.PREPFUEL;
            }
            break;
          case SHOOT:
            runShooter();
            if (MathUtil.isNear(targetRPM, getShooterSpeed(), 5)) {
              currShooterState = ShooterState.SHOOT;
            }
            break;
          default:
            break;
        }
    }
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    handleStateTransition();

    DogLog.log("Shooter/Shooter State", currShooterState.toString());

    shooterLeadIO.updateInputs(shooterLeadInputs);
    shooterFollowIO.updateInputs(shooterFollowInputs);
  }
}
