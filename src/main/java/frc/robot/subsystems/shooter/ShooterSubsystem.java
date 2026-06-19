package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.constants.ShooterConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
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
    // PURGE,
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
    this.targetRPM = ShooterConstants.SHOOTER_RPM;
    this.hoodAngle = ShooterConstants.SHOOTER_HOOD_DEFAULT_SETTING; // default to home position
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

  public void runShooterMotorVoltage(Voltage voltage) {
    shooterLeadIO.setMotorVoltage(voltage);
  }

  public void runShooterMotorRPM(double rpm) {
    shooterLeadIO.setMotorRPM(rpm);
  }

  public void runShooter() {
    runShooterMotorRPM(targetRPM);
  }

  // runs the shooter at half speed
  public void prepShooter() {
    runShooterMotorRPM(1200); // Run at 50% of target RPM for prep
  }

  public void stopShooter() {
    runShooterMotorVoltage(Volts.of(0));
  }

  public void runKickerMotorPercentage(double percentage) {
    shooterKickerIO.setMotorPercentage(percentage);
  }

  public void runKickerMotorVoltage(Voltage voltage) {
    shooterKickerIO.setMotorVoltage(voltage);
  }

  public void runKicker() {
    runKickerMotorVoltage(
        ShooterConstants.SHOOTER_KICKER_VOLTAGE); // Run kicker at full voltage for shooting
  }

  public void prepKicker() {
    runKickerMotorVoltage(
        ShooterConstants.SHOOTER_KICKER_PREP_VOLTAGE); // Run kicker at 50% of full voltage for prep
  }

  public void stopKicker() {
    runKickerMotorVoltage(Volts.of(0));
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

  public void setSetpoint(double shooterRPM, double hoodAngle) {
    targetRPM = shooterRPM;
    this.hoodAngle = hoodAngle;
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
        case STOP -> currShooterState = ShooterState.TRANSITION;
        case PREPFUEL -> currShooterState = ShooterState.TRANSITION;
        case SHOOT -> currShooterState = ShooterState.TRANSITION;
        default -> {}
      }
    }
  }

  public void handleStateTransition() {
    switch (currShooterState) {
      case STOP -> {
        // Only send CAN commands while kicker timer is active;
        // once fully stopped, motors hold their last command — no need to re-send every
        // loop
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
        // Motors are already stopped from the TRANSITION→STOP path; no redundant writes
        // needed
      }
      case PREPFUEL -> {
        kickerStopTimerRunning = false;
        kickerStopTimer.stop();
        kickerStopTimer.reset();
        prepShooter();
        prepKicker();
      }
      case SHOOT -> {
        runShooter();
        runKicker();
        setHoodAngle(hoodAngle);
      }
      case TRANSITION -> {
        switch (desiredShooterState) {
          case STOP:
            stopShooter();
            if (!kickerStopTimerRunning) {
              kickerStopTimer.reset();
              kickerStopTimer.start();
              kickerStopTimerRunning = true;
            }
            if (kickerStopTimerRunning && !kickerStopTimer.hasElapsed(KICKER_STOP_DELAY)) {
              runKickerMotorPercentage(0.5);
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
          default:
            break;
        }
      }
    }
  }

  @Override
  public void periodic() {
    shooterLeadIO.updateInputs(shooterLeadInputs);
    shooterFollowIO.updateInputs(shooterFollowInputs);
    shooterKickerIO.updateInputs(shooterKickerInputs);
    shooterHoodIO.updateInputs(shooterHoodInputs);

    handleStateTransition();

    DogLog.log("Shooter/CurrentState", currShooterState.toString());
    DogLog.log("Shooter/DesiredState", desiredShooterState.toString());
    DogLog.log("Shooter/SpeedRPM", getShooterSpeed());
    DogLog.log("Shooter/TargetRPM", targetRPM);
    DogLog.log("Shooter/HoodAngle", hoodAngle);
  }
}
