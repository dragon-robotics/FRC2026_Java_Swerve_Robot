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
    TRANSITION // Transition state to handle ramping up/down of the shooter speed
  }

  private ShooterState desiredShooterState;
  private ShooterState currShooterState;

  private ShooterIO shooterHoodIO;
  private ShooterIO shooterKickerIO;
  private ShooterIO shooterLeadIO, shooterFollowIO;
  private ShooterIOInputs shooterLeadInputs, shooterFollowInputs;

  private double targetRPM;
  private double hoodAngle;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(
      ShooterIO shooterHoodIO,
      ShooterIO shooterKickerIO,
      ShooterIO shooterLeadIO,
      ShooterIO shooterFollowIO) {
    this.shooterHoodIO = shooterHoodIO;
    this.shooterKickerIO = shooterKickerIO;
    this.shooterLeadIO = shooterLeadIO;
    this.shooterFollowIO = shooterFollowIO;
    this.shooterLeadInputs = new ShooterIOInputs();
    this.shooterFollowInputs = new ShooterIOInputs();

    // initialize shooter states
    this.desiredShooterState = ShooterState.STOP;
    this.currShooterState = ShooterState.STOP;

    // Initialize target RPM and hood angle to default values
    this.targetRPM = 0;
    this.hoodAngle = 0;
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
    shooterKickerIO.setMotorRPM(0);
  }

  public void setHoodAngle() {
    shooterHoodIO.setMotorPosition(hoodAngle);
  }

  public void setSetpointForDistance(double distanceToTarget) {
    ShooterSetpoint setpoint = getSetpointForDistance(distanceToTarget);
    targetRPM = setpoint.shooterRPM();
    hoodAngle = setpoint.hoodAngleDeg();
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
            if (isShooterStopped()) {
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

    // Set Hood Angle every loop to ensure it reaches the desired position
    setHoodAngle();

    // This method will be called once per scheduler run
    handleStateTransition();

    DogLog.log("Shooter/Shooter State", currShooterState.toString());

    shooterLeadIO.updateInputs(shooterLeadInputs);
    shooterFollowIO.updateInputs(shooterFollowInputs);
  }
}
