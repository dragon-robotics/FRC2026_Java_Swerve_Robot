package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_HOOD_DEFAULT_SETTING;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_HOOD_READY_TOLERANCE_ROTATIONS;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_KICKER_PREP_VOLTAGE;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_KICKER_STOP_DELAY_SECONDS;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_KICKER_STOP_PERCENT_OUTPUT;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_KICKER_VOLTAGE;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_PREP_RPM;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_READY_TOLERANCE_RPM;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_RPM;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_STOPPED_TOLERANCE_RPM;
import static frc.robot.util.constants.ShooterConstants.getSetpointForDistance;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIO.MotorIOInputs;
import frc.robot.util.constants.ShooterConstants.ShooterSetpoint;

/**
 * Controls the shooter flywheel, kicker, and adjustable hood.
 *
 * <p>Flywheel commands are RPM or volts depending on the method name. Hood setpoints are mechanism
 * rotations from the hood encoder. The kicker keeps feeding briefly when stopping so fuel clears
 * the shooter path.
 */
public class ShooterSubsystem extends SubsystemBase {
  /** Shooter state machine states requested by the superstructure. */
  public enum ShooterState {
    STOP,
    PREPFUEL,
    SHOOT,
    TRANSITION
  }

  protected ShooterState desiredShooterState;
  protected ShooterState currShooterState;

  protected final MotorIO shooterHoodIO;
  protected final MotorIO shooterKickerIO;
  protected final MotorIO shooterLeadIO;
  protected final MotorIO shooterFollowIO;
  protected final MotorIOInputs shooterLeadInputs;
  protected final MotorIOInputs shooterFollowInputs;
  protected final MotorIOInputs shooterKickerInputs;
  protected final MotorIOInputs shooterHoodInputs;

  protected double targetRPM;
  protected double hoodAngle;

  /** Keeps the kicker running after STOP is requested so already-fed fuel clears the shooter. */
  private final Timer kickerStopTimer = new Timer();

  private boolean kickerStopTimerRunning = false;

  /**
   * Creates a new shooter subsystem.
   *
   * @param shooterLeadIO lead flywheel motor IO; receives flywheel commands
   * @param shooterFollowIO follower flywheel motor IO; updated for telemetry
   * @param shooterKickerIO kicker motor IO; feeds fuel into the flywheel
   * @param shooterHoodIO hood motor IO; controls hood position in mechanism rotations
   */
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

    this.desiredShooterState = ShooterState.STOP;
    this.currShooterState = ShooterState.STOP;
    this.targetRPM = SHOOTER_RPM;
    this.hoodAngle = SHOOTER_HOOD_DEFAULT_SETTING;
  }

  /** Returns the current state being executed by the shooter state machine. */
  public ShooterState getCurrentState() {
    return this.currShooterState;
  }

  /** Returns the requested shooter state. Name kept for compatibility with existing callers. */
  public ShooterState getShooterState() {
    return this.desiredShooterState;
  }

  /** Directly commands the lead flywheel percent output from -1.0 to 1.0. */
  public void runShooterMotorPercentage(double percentage) {
    shooterLeadIO.setMotorPercentage(percentage);
  }

  /** Directly commands the lead flywheel voltage. */
  public void runShooterMotorVoltage(Voltage voltage) {
    shooterLeadIO.setMotorVoltage(voltage);
  }

  /** Directly commands the lead flywheel velocity in RPM. */
  public void runShooterMotorRPM(double rpm) {
    shooterLeadIO.setMotorRPM(rpm);
  }

  /** Runs the flywheel at the currently selected target RPM. */
  public void runShooter() {
    runShooterMotorRPM(targetRPM);
  }

  /** Runs the flywheel at the configured prep RPM. */
  public void prepShooter() {
    runShooterMotorRPM(SHOOTER_PREP_RPM);
  }

  /** Stops the flywheel with 0 volts. */
  public void stopShooter() {
    runShooterMotorVoltage(Volts.of(0));
  }

  /** Directly commands the kicker percent output from -1.0 to 1.0. */
  public void runKickerMotorPercentage(double percentage) {
    shooterKickerIO.setMotorPercentage(percentage);
  }

  /** Directly commands the kicker voltage. */
  public void runKickerMotorVoltage(Voltage voltage) {
    shooterKickerIO.setMotorVoltage(voltage);
  }

  /** Runs the kicker at the configured shooting voltage. */
  public void runKicker() {
    runKickerMotorVoltage(SHOOTER_KICKER_VOLTAGE);
  }

  /** Runs the kicker at the configured prep voltage. */
  public void prepKicker() {
    runKickerMotorVoltage(SHOOTER_KICKER_PREP_VOLTAGE);
  }

  /** Stops the kicker with 0 volts. */
  public void stopKicker() {
    runKickerMotorVoltage(Volts.of(0));
  }

  /* Setters */

  /**
   * Commands the hood to a mechanism position.
   *
   * @param position hood mechanism rotations from the hood encoder
   */
  public void setHoodAngle(double position) {
    shooterHoodIO.setMotorPosition(position);
  }

  /**
   * Selects interpolated flywheel RPM and hood rotations for a target distance.
   *
   * @param distanceMeters distance to target in meters
   */
  public void setSetpointForDistance(double distanceMeters) {
    ShooterSetpoint setpoint = getSetpointForDistance(distanceMeters);
    targetRPM = setpoint.shooterRPM();
    hoodAngle = setpoint.hoodAngle();
  }

  /**
   * Directly selects the shooter target.
   *
   * @param shooterRPM flywheel target in RPM
   * @param hoodAngle hood target in mechanism rotations
   */
  public void setSetpoint(double shooterRPM, double hoodAngle) {
    targetRPM = shooterRPM;
    this.hoodAngle = hoodAngle;
  }

  /** Returns the lead flywheel velocity in RPM. */
  public double getShooterSpeed() {
    return shooterLeadInputs.getMotorVelocity() * 60.0;
  }

  /** Returns true when the flywheel is moving fast enough to be considered active. */
  public boolean isShooting() {
    return getShooterSpeed() > SHOOTER_READY_TOLERANCE_RPM;
  }

  /** Returns true when the flywheel speed is within the stopped tolerance in RPM. */
  public boolean isShooterStopped() {
    return Math.abs(getShooterSpeed()) < SHOOTER_STOPPED_TOLERANCE_RPM;
  }

  /** Returns the selected flywheel target in RPM. */
  public double getTargetRPM() {
    return targetRPM;
  }

  /* State Management */

  /**
   * Requests a shooter state.
   *
   * <p>Closed-loop transitions run through {@link ShooterState#TRANSITION} before the requested
   * steady state is reached.
   */
  public void setDesiredState(ShooterState state) {
    this.desiredShooterState = state;

    if (desiredShooterState != currShooterState) {
      switch (desiredShooterState) {
        case STOP, PREPFUEL, SHOOT -> currShooterState = ShooterState.TRANSITION;
        default -> {}
      }
    }
  }

  /** Advances the shooter state machine and sends hardware commands for the current state. */
  public void handleStateTransition() {
    switch (currShooterState) {
      case STOP -> handleStopState();
      case PREPFUEL -> handlePrepFuelState();
      case SHOOT -> handleShootState();
      case TRANSITION -> handleTransitionState();
    }
  }

  private void handleStopState() {
    if (!kickerStopTimerRunning) {
      return;
    }

    if (kickerStopTimer.hasElapsed(SHOOTER_KICKER_STOP_DELAY_SECONDS)) {
      stopKicker();
      kickerStopTimerRunning = false;
      kickerStopTimer.stop();
      kickerStopTimer.reset();
    } else {
      runKicker();
    }
  }

  private void handlePrepFuelState() {
    clearKickerStopTimer();
    prepShooter();
    prepKicker();
  }

  private void handleShootState() {
    runShooter();
    runKicker();
    setHoodAngle(hoodAngle);
  }

  private void handleTransitionState() {
    switch (desiredShooterState) {
      case STOP -> transitionToStop();
      case PREPFUEL -> transitionToPrepFuel();
      case SHOOT -> transitionToShoot();
      default -> {}
    }
  }

  private void transitionToStop() {
    stopShooter();
    startKickerStopTimerIfNeeded();

    if (kickerStopTimerRunning && !kickerStopTimer.hasElapsed(SHOOTER_KICKER_STOP_DELAY_SECONDS)) {
      runKickerMotorPercentage(SHOOTER_KICKER_STOP_PERCENT_OUTPUT);
    } else {
      stopKicker();
    }

    setHoodAngle(SHOOTER_HOOD_DEFAULT_SETTING);
    if (isShooterStopped()) {
      currShooterState = ShooterState.STOP;
    }
  }

  private void transitionToPrepFuel() {
    prepShooter();
    prepKicker();
    setHoodAngle(SHOOTER_HOOD_DEFAULT_SETTING);

    if (MathUtil.isNear(SHOOTER_PREP_RPM, getShooterSpeed(), SHOOTER_READY_TOLERANCE_RPM)) {
      currShooterState = ShooterState.PREPFUEL;
    }
  }

  private void transitionToShoot() {
    runShooter();
    setHoodAngle(hoodAngle);

    if (MathUtil.isNear(targetRPM, getShooterSpeed(), SHOOTER_READY_TOLERANCE_RPM)
        && MathUtil.isNear(
            hoodAngle,
            shooterHoodInputs.getMotorPosition(),
            SHOOTER_HOOD_READY_TOLERANCE_ROTATIONS)) {
      currShooterState = ShooterState.SHOOT;
    }
  }

  private void startKickerStopTimerIfNeeded() {
    if (kickerStopTimerRunning) {
      return;
    }

    kickerStopTimer.reset();
    kickerStopTimer.start();
    kickerStopTimerRunning = true;
  }

  private void clearKickerStopTimer() {
    kickerStopTimerRunning = false;
    kickerStopTimer.stop();
    kickerStopTimer.reset();
  }

  @Override
  public void periodic() {
    shooterLeadIO.updateInputs(shooterLeadInputs);
    shooterFollowIO.updateInputs(shooterFollowInputs);
    shooterKickerIO.updateInputs(shooterKickerInputs);
    shooterHoodIO.updateInputs(shooterHoodInputs);

    handleStateTransition();

    DogLog.forceNt.log("Shooter/CurrentState", currShooterState.toString());
    DogLog.log("Shooter/DesiredState", desiredShooterState.toString());
    DogLog.log("Shooter/SpeedRPM", getShooterSpeed());
    DogLog.log("Shooter/TargetRPM", targetRPM);
    DogLog.log("Shooter/HoodAngle", hoodAngle);
  }
}
