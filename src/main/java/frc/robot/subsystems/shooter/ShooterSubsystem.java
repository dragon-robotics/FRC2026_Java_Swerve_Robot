package frc.robot.subsystems.shooter;

import static frc.robot.util.constants.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIO.MotorIOInputs;
import frc.robot.util.constants.ShooterConstants;
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
  protected final MotorIOInputs shooterLeadInputs,
      shooterFollowInputs,
      shooterKickerInputs,
      shooterHoodInputs;

  protected double targetRPM;
  protected double hoodAngle;

  // Timer to keep kicker running after shooting stops
  private final Timer kickerStopTimer = new Timer();
  private boolean kickerStopTimerRunning = false;
  private static final double KICKER_STOP_DELAY = 1.0; // seconds

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
    this.hoodAngle = 0.0; // default to home position
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
    // Kicker is managed separately by stopKicker()/runKicker() — don't send conflicting commands
  }

  public void runKicker() {
    shooterKickerIO.setMotorPercentage(0.75); // Run kicker at 75% of full RPM for shooting
  }

  public void prepKicker() {
    shooterKickerIO.setMotorPercentage(0.15); // Run kicker at 30% of full RPM for prep
  }

  public void stopKicker() {
    shooterKickerIO.setMotorPercentage(0);
  }

  /* Setters */

  public void setHoodAngle(double position) {
    shooterHoodIO.setMotorPosition(position);
  }

  public void setSetpointForDistance(double distanceToTarget) {
    ShooterSetpoint setpoint = getSetpointForDistance(distanceToTarget);
    targetRPM = setpoint.shooterRPM();
    hoodAngle = setpoint.hoodAngle();
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

  /* State Management */

  public void setDesiredState(ShooterState state) {
    this.desiredShooterState = state;

    if (desiredShooterState != currShooterState) {
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
  }

  public void handleStateTransition() {
    switch (currShooterState) {
      case STOP:
        // Only send CAN commands while kicker timer is active;
        // once fully stopped, motors hold their last command — no need to re-send every loop
        if (kickerStopTimerRunning) {
          if (kickerStopTimer.hasElapsed(KICKER_STOP_DELAY)) {
            stopKicker();
            kickerStopTimerRunning = false;
            kickerStopTimer.stop();
            kickerStopTimer.reset();
          } else {
            runKicker();
          }
        }
        // Motors are already stopped from the TRANSITION→STOP path; no redundant writes needed
        break;
      case PREPFUEL:
        kickerStopTimerRunning = false;
        kickerStopTimer.stop();
        kickerStopTimer.reset();
        prepShooter();
        prepKicker();
        break;
      case SHOOT:
        runShooter();
        runKicker();
        setHoodAngle(hoodAngle);
        break;
      case TRANSITION:
        switch (desiredShooterState) {
          case STOP:
            stopShooter();
            if (!kickerStopTimerRunning) {
              kickerStopTimer.reset();
              kickerStopTimer.start();
              kickerStopTimerRunning = true;
            }
            if (kickerStopTimerRunning && !kickerStopTimer.hasElapsed(KICKER_STOP_DELAY)) {
              runKicker();
            } else {
              stopKicker();
            }
            setHoodAngle(0);
            if (isShooterStopped()) {
              currShooterState = ShooterState.STOP;
            }
            break;
          case PREPFUEL:
            prepShooter();
            prepKicker();
            setHoodAngle(0);
            if (MathUtil.isNear(targetRPM * 0.3, getShooterSpeed(), 60)) {
              currShooterState = ShooterState.PREPFUEL;
            }
            break;
          case SHOOT:
            runShooter();
            setHoodAngle(hoodAngle);
            if (MathUtil.isNear(targetRPM, getShooterSpeed(), 60)
                && MathUtil.isNear(hoodAngle, shooterHoodInputs.getMotorPosition(), 0.125)) {
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
    shooterLeadIO.updateInputs(shooterLeadInputs);
    // shooterFollowIO.updateInputs(shooterFollowInputs);
    // shooterKickerIO.updateInputs(shooterKickerInputs);
    shooterHoodIO.updateInputs(shooterHoodInputs);

    handleStateTransition();
  }
}
