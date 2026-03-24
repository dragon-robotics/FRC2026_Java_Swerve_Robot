package frc.robot.subsystems.shooter;

import static frc.robot.util.constants.ShooterConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIO.MotorIOInputs;
import frc.robot.util.constants.ShooterConstants;
import frc.robot.util.constants.ShooterConstants.ShooterHoodSettings;
import frc.robot.util.constants.ShooterConstants.ShooterSetpoint;

public class ShooterSubsystem extends SubsystemBase {
  public enum ShooterState {
    STOP,
    PREPFUEL,
    SHOOT,
    TRANSITION // Transition state to handle ramping up/down of the shooter speed
  }

  protected ShooterState desiredShooterState;
  protected ShooterState currShooterState;

  protected final MotorIO shooterHoodIO;
  protected final MotorIO shooterKickerIO;
  protected final MotorIO shooterLeadIO, shooterFollowIO;
  protected final MotorIOInputs shooterLeadInputs, shooterFollowInputs, shooterKickerInputs, shooterHoodInputs;

  protected double targetRPM;
  protected ShooterHoodSettings hoodSetting;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(
      MotorIO shooterLeadIO,
      MotorIO shooterFollowIO,
      MotorIO shooterKickerIO,
      MotorIO shooterHoodIO) {
    this.shooterLeadIO = shooterLeadIO;
    this.shooterFollowIO = shooterFollowIO;
    this.shooterKickerIO = shooterKickerIO;
    this.shooterHoodIO = shooterHoodIO;
    this.shooterLeadInputs = new MotorIOInputs();
    this.shooterFollowInputs = new MotorIOInputs();
    this.shooterKickerInputs = new MotorIOInputs();
    this.shooterHoodInputs = new MotorIOInputs();

    // initialize shooter states
    this.desiredShooterState = ShooterState.STOP;
    this.currShooterState = ShooterState.STOP;

    // Initialize target RPM and hood angle to default values
    this.targetRPM = ShooterConstants.SHOOTER_LEAD_RPM;
    this.hoodSetting = ShooterHoodSettings.HOME;
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

  public void runShooterMotorPercentage(double percentage) {
    shooterLeadIO.setMotorPercentage(percentage);
  }

  public void runKickerMotorPercentage(double percentage) {
    shooterKickerIO.setMotorPercentage(percentage);
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

  public void runKicker() {
    // shooterKickerIO.setMotorRPM(SHOOTER_KICKER_RPM);
    shooterKickerIO.setMotorPercentage(0.75); // Run kicker at full RPM for shooting
  }

  public void prepKicker() {
    // shooterKickerIO.setMotorRPM(SHOOTER_KICKER_RPM * 0.3); // Run at 30% of kicker RPM for prep
    shooterKickerIO.setMotorPercentage(0.75 * 0.3); // Run kicker at 30% of full RPM for prep
  }

  public void stopKicker() {
    // shooterKickerIO.setMotorRPM(0);
    shooterKickerIO.setMotorPercentage(0);
  }

  public void setHoodAngle(double position) {
    shooterHoodIO.setMotorPosition(position);
  }

  public void setSetpointForDistance(double distanceToTarget) {
    ShooterSetpoint setpoint = getSetpointForDistance(distanceToTarget);
    targetRPM = setpoint.shooterRPM();

    /* TODO: Switch based on distance and where we are on the field */
    hoodSetting = ShooterHoodSettings.HOME;
  }

  /* Returns the speed in RPM */
  public double getShooterSpeed() {
    return shooterLeadInputs.getMotorVelocity() * 60.0; // Convert from RPS to RPM
  }

  public boolean isShooting() {
    return getShooterSpeed() > 60;
  }

  public boolean isShooterStopped() {
    return Math.abs(getShooterSpeed()) < 0.5; // Use threshold instead of exact comparison
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public void handleStateTransition() {
    // Handle the state transitions
    switch (currShooterState) {
      case STOP:
        stopShooter();
        stopKicker();
        setHoodAngle(0);
        break;
      case PREPFUEL:
        prepShooter();
        break;
      case SHOOT:
        runShooter();
        runKicker();
        setHoodAngle(1.25);
        break;
      case TRANSITION:
        switch (desiredShooterState) {
          case STOP:
            stopShooter();
            stopKicker();
            setHoodAngle(0);
            if (isShooterStopped()) {
              currShooterState = ShooterState.STOP;
            }
            break;
          case PREPFUEL:
            prepShooter();
            if (MathUtil.isNear(targetRPM * 0.3, getShooterSpeed(), 60)) {
              currShooterState = ShooterState.PREPFUEL;
            }
            break;
          case SHOOT:
            runShooter();
            runKicker();
            setHoodAngle(1.25);
            if (MathUtil.isNear(targetRPM, getShooterSpeed(), 60) && MathUtil.isNear(1.25, shooterHoodInputs.getMotorPosition(), 0.1)) {
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
    // setHoodAngle();

    // This method will be called once per scheduler run
    handleStateTransition();

    DogLog.log("Shooter/Shooter State", currShooterState.toString());

    shooterLeadIO.updateInputs(shooterLeadInputs);
    shooterFollowIO.updateInputs(shooterFollowInputs);
  }
}
