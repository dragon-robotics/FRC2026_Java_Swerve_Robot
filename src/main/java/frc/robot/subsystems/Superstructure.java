package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.AimAtTargetPoseCmd;
import frc.robot.commands.DefaultDriveCmd;
import frc.robot.commands.ShootDriveCmd;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem.HopperState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.HubShiftUtil.ShiftInfo;
import frc.robot.util.Telemetry;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;

/**
 * Coordinates drivetrain, intake, hopper, shooter, and vision into
 * driver-facing robot states.
 *
 * <p>
 * This subsystem owns cross-subsystem command factories and telemetry only.
 * Mechanism commands
 * still require the individual subsystems so WPILib scheduling can handle
 * preemption correctly.
 */
public class Superstructure extends SubsystemBase {

  /** Driver-facing robot modes composed from individual subsystem states. */
  public enum SuperState {
    DRIVE_STARTING_CONFIG,
    DRIVE,
    INTAKE,
    OUTTAKE,
    /** Shooting while aiming with vision assistance. */
    SHOOT_WITH_AIM,
    /**
     * Shooting without aiming assistance while still using distance-based shooter
     * setpoints.
     */
    SHOOT_NO_AIM,
    /** Shooting with fixed manual shooter setpoints. */
    MANUAL_SHOOT,
    /** Shooting while outtaking to clear fuel quickly. */
    PURGE
  }

  /** Selects which command runs when the driver requests a shot. */
  public enum ShootMode {
    DEFAULT_SHOOT_WITH_AIM,
    MANUAL_BUMPER_UP,
    MANUAL_TRENCH
  }

  /* Subsystem references */

  private final CommandSwerveDrivetrain swerve;
  private final IntakeSubsystem intake;
  private final HopperSubsystem hopper;
  private final ShooterSubsystem shooter;
  private final VisionSubsystem vision;
  private final RobotContainer container;
  private final Telemetry logger;

  /* Swerve requests */

  private final SwerveRequest.SwerveDriveBrake brake;
  private final SwerveRequest.PointWheelsAt point;
  private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds;
  private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds;

  /* Heading tracking */

  private Optional<Rotation2d> currentHeading;
  private double rotationLastTriggered;

  /* Alignment and targeting */

  private static final double ALIGNMENT_TOLERANCE_DEGREES = 5;
  private boolean alignedToTarget = false;
  private boolean allianceConfirmed = false;
  private FieldZones currentZone;
  private ShootMode shootMode = ShootMode.DEFAULT_SHOOT_WITH_AIM;

  /* Manual shot setpoints */
  private static final double MANUAL_BUMPER_UP_RPM = 2500.0;
  private static final double MANUAL_BUMPER_UP_HOOD = 0.0;
  private static final double MANUAL_TRENCH_RPM = 2900.0;
  private static final double MANUAL_TRENCH_HOOD = 0.75;
  private static final double NEUTRAL_ZONE_HOOD_LOCK = 2.0;

  /** Pre-cached zone names avoid .name() heap allocation every cycle. */
  private static final String[] ZONE_NAMES;

  static {
    FieldZones[] zones = FieldZones.values();
    ZONE_NAMES = new String[zones.length];
    for (int i = 0; i < zones.length; i++) {
      ZONE_NAMES[i] = zones[i].name();
    }
  }

  /* State machine */
  private SuperState state;

  /* Alliance state */
  private DriverStation.Alliance alliance;

  public Superstructure(
      CommandSwerveDrivetrain swerve,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      ShooterSubsystem shooter,
      VisionSubsystem vision,
      RobotContainer container) {
    this.swerve = swerve;
    this.intake = intake;
    this.hopper = hopper;
    this.shooter = shooter;
    this.vision = vision;
    this.container = container;

    state = SuperState.DRIVE_STARTING_CONFIG;

    brake = new SwerveRequest.SwerveDriveBrake();
    point = new SwerveRequest.PointWheelsAt();
    applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(DriveRequestType.Velocity);
    applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(DriveRequestType.Velocity);

    currentHeading = Optional.empty();
    rotationLastTriggered = 0.0;

    logger = new Telemetry();
    swerve.registerTelemetry(logger::telemeterize);

    /* Default to blue until Driver Station alliance becomes available. */
    alliance = DriverStation.Alliance.Blue;
    currentZone = null;
    refreshAllianceAndCachedHubTarget();

    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    intake.setDefaultCommand(
        intake
            .runOnce(() -> intake.setDesiredState(IntakeState.DEPLOYED))
            .withName("Intake.Default(DEPLOYED)"));
    hopper.setDefaultCommand(
        hopper
            .runOnce(() -> hopper.setDesiredState(HopperState.STOP))
            .withName("Hopper.Default(STOP)"));
    shooter.setDefaultCommand(
        shooter
            .runOnce(() -> shooter.setDesiredState(ShooterState.PREPFUEL))
            .withName("Shooter.Default(PREPFUEL)"));
  }

  private void setAlliance(DriverStation.Alliance newAlliance) {
    alliance = newAlliance;
  }

  static Translation2d resolveAimTargetForZone(
      boolean allianceConfirmed, FieldZones zone, DriverStation.Alliance alliance) {
    if (!allianceConfirmed || zone == null) {
      return alliance == DriverStation.Alliance.Red
          ? FieldConstants.Hub.RED_CENTER_POSE
          : FieldConstants.Hub.BLUE_CENTER_POSE;
    }

    boolean isRed = alliance == DriverStation.Alliance.Red;
    return switch (zone) {
      // Zone labels and aim points are both blue-relative after alliance
      // normalization,
      // so red mappings should stay LEFT->LEFT and RIGHT->RIGHT here.
      case NEUTRAL_LEFT_SHOOT -> isRed
          ? FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT
          : FieldConstants.AimPoints.BLUE_LEFT_SHOOT_POINT;
      case NEUTRAL_RIGHT_SHOOT -> isRed
          ? FieldConstants.AimPoints.RED_RIGHT_SHOOT_POINT
          : FieldConstants.AimPoints.BLUE_RIGHT_SHOOT_POINT;
      case NEUTRAL_LEFT_PURGE -> isRed
          ? FieldConstants.AimPoints.RED_LEFT_PURGE_POINT
          : FieldConstants.AimPoints.BLUE_LEFT_PURGE_POINT;
      case NEUTRAL_RIGHT_PURGE -> isRed
          ? FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT
          : FieldConstants.AimPoints.BLUE_RIGHT_PURGE_POINT;
      default -> isRed ? FieldConstants.Hub.RED_CENTER_POSE : FieldConstants.Hub.BLUE_CENTER_POSE;
    };
  }

  private Translation2d getCurrentAimTarget() {
    return resolveAimTargetForZone(allianceConfirmed, currentZone, alliance);
  }

  private boolean isShootAllowedZone() {
    if (!allianceConfirmed || currentZone == null) {
      return false;
    }

    return switch (currentZone) {
      case ALLIANCE_LEFT,
          ALLIANCE_RIGHT,
          ALLIANCE_LEFT_TRENCH,
          ALLIANCE_RIGHT_TRENCH,
          NEUTRAL_LEFT_SHOOT,
          NEUTRAL_RIGHT_SHOOT,
          NEUTRAL_LEFT_PURGE,
          NEUTRAL_RIGHT_PURGE ->
        true;
      default -> false;
    };
  }

  private boolean isPurgeZone() {
    if (!allianceConfirmed || currentZone == null) {
      return false;
    }

    return switch (currentZone) {
      case NEUTRAL_LEFT_PURGE, NEUTRAL_RIGHT_PURGE -> true;
      default -> false;
    };
  }

  private boolean isNeutralShootOrPurgeZone() {
    if (!allianceConfirmed || currentZone == null) {
      return false;
    }

    return switch (currentZone) {
      case NEUTRAL_LEFT_SHOOT, NEUTRAL_RIGHT_SHOOT, NEUTRAL_LEFT_PURGE, NEUTRAL_RIGHT_PURGE -> true;
      default -> false;
    };
  }

  private Command createShootStateCommand(boolean withAim) {
    Command shootCommand = Commands.run(
        () -> {
          setDesiredSuperState(withAim ? SuperState.SHOOT_WITH_AIM : SuperState.SHOOT_NO_AIM);
          shooter.setDesiredState(ShooterState.SHOOT);
          setHopperFeedWhenShooterReady(true);
        },
        shooter,
        hopper);

    if (!withAim) {
      return shootCommand;
    }

    return shootCommand.alongWith(
        new AimAtTargetPoseCmd(swerve, this::setCurrentHeading, this::getCurrentAimTarget));
  }

  private Command createPurgeStateCommand() {
    return Commands.run(
        () -> {
          setDesiredSuperState(SuperState.PURGE);
          intake.setDesiredState(IntakeState.OUTTAKE);
          shooter.setDesiredState(ShooterState.SHOOT);
          setHopperFeedWhenShooterReady(true);
        },
        intake,
        shooter,
        hopper)
        .alongWith(
            new AimAtTargetPoseCmd(swerve, this::setCurrentHeading, this::getCurrentAimTarget));
  }

  private Command createManualShootStateCommand(double shooterRpm, double hoodAngle) {
    return Commands.run(
        () -> {
          shooter.setSetpoint(shooterRpm, hoodAngle);
          setDesiredSuperState(SuperState.MANUAL_SHOOT);
          shooter.setDesiredState(ShooterState.SHOOT);
          setHopperFeedWhenShooterReady(false);
        },
        shooter,
        hopper)
        .alongWith(Commands.run(() -> swerve.setControl(brake), swerve))
        .withName("SuperState(MANUAL_SHOOT)");
  }

  private void setHopperFeedWhenShooterReady(boolean requireAlignment) {
    boolean shooterReady = shooter.getCurrentState() == ShooterState.SHOOT;
    boolean alignmentReady = !requireAlignment || isAlignedToTarget();
    hopper.setDesiredState(
        shooterReady && alignmentReady ? HopperState.INDEXTOSHOOTER : HopperState.STOP);
  }

  /**
   * Shoot with aim, transitioning intake to JUICER after 1.5 s. Owns intake,
   * shooter, hopper, and
   * swerve in a single command group so there is no parallel-requirements
   * conflict. Intended for
   * autonomous use.
   */
  public Command shootWithJuicerDelayCmd() {
    Timer juicerTimer = new Timer();
    return Commands.runOnce(juicerTimer::restart)
        .andThen(
            Commands.run(
                () -> {
                  boolean purge = isPurgeZone();
                  setDesiredSuperState(purge ? SuperState.PURGE : SuperState.SHOOT_WITH_AIM);
                  if (purge) {
                    intake.setDesiredState(IntakeState.OUTTAKE);
                  } else {
                    intake.setDesiredState(
                        juicerTimer.hasElapsed(1.5)
                            ? IntakeState.JUICER
                            : IntakeState.DEPLOYED);
                  }
                  shooter.setDesiredState(ShooterState.SHOOT);
                  setHopperFeedWhenShooterReady(true);
                },
                intake,
                shooter,
                hopper)
                .alongWith(
                    new AimAtTargetPoseCmd(
                        swerve, this::setCurrentHeading, this::getCurrentAimTarget)))
        .withName("SuperState(SHOOT_WITH_AIM+Juicer)");
  }

  /**
   * Shoot without drivetrain aiming, transitioning intake to JUICER after 1.5 s.
   * Owns intake,
   * shooter, and hopper so autonomous event markers do not need a parallel intake
   * override.
   */
  public Command shootNoAimWithJuicerDelayCmd() {
    Timer juicerTimer = new Timer();
    return Commands.runOnce(juicerTimer::restart)
        .andThen(
            Commands.run(
                () -> {
                  setDesiredSuperState(SuperState.SHOOT_NO_AIM);
                  intake.setDesiredState(
                      juicerTimer.hasElapsed(1.5) ? IntakeState.JUICER : IntakeState.DEPLOYED);
                  shooter.setDesiredState(ShooterState.SHOOT);
                  setHopperFeedWhenShooterReady(true);
                },
                intake,
                shooter,
                hopper))
        .withName("SuperState(SHOOT_NO_AIM+Juicer)");
  }

  public Command selectedShootModeCmd() {
    return switch (shootMode) {
      case DEFAULT_SHOOT_WITH_AIM -> {
        // Driver-triggered default shoot should not claim intake so JUICER override
        // can run concurrently outside purge zones.
        if (!isShootAllowedZone()) {
          yield Commands.idle().withName("SuperState(SHOOT_WITH_AIM:DISALLOWED)");
        }
        yield createShootStateCommand(true).withName("SuperState(SHOOT_WITH_AIM)");
      }
      case MANUAL_BUMPER_UP -> createManualShootStateCommand(
          MANUAL_BUMPER_UP_RPM, MANUAL_BUMPER_UP_HOOD)
          .withName("SuperState(SHOOT->MANUAL_BUMPER_UP)");
      case MANUAL_TRENCH -> createManualShootStateCommand(MANUAL_TRENCH_RPM, MANUAL_TRENCH_HOOD)
          .withName("SuperState(SHOOT->MANUAL_TRENCH)");
    };
  }

  /** True when right-trigger shooting should switch to purge behavior. */
  public boolean shouldUsePurgeDuringShoot() {
    return shootMode == ShootMode.DEFAULT_SHOOT_WITH_AIM && isPurgeZone();
  }

  /** Driver-triggered purge command that explicitly requires intake. */
  public Command purgeShootCmd() {
    if (!isPurgeZone()) {
      return Commands.idle().withName("SuperState(SHOOT->PURGE:DISALLOWED)");
    }
    return createPurgeStateCommand().withName("SuperState(SHOOT->PURGE)");
  }

  private void refreshAllianceAndCachedHubTarget() {
    DriverStation.getAlliance()
        .ifPresent(
            dsAlliance -> {
              boolean allianceChanged = dsAlliance != alliance;
              setAlliance(dsAlliance);
              allianceConfirmed = true;
              if (allianceChanged) {
                DogLog.log("Superstructure/AllianceConfirmed", dsAlliance.name());
              }
            });
  }

  /* Heading accessors used by DefaultDriveCmd */

  public Optional<Rotation2d> getCurrentHeading() {
    return currentHeading;
  }

  public void setCurrentHeading(Optional<Rotation2d> heading) {
    this.currentHeading = heading;
  }

  public double getRotationLastTriggered() {
    return rotationLastTriggered;
  }

  public void setRotationLastTriggered(double t) {
    this.rotationLastTriggered = t;
  }

  /* Drive command factories */

  public Command defaultDrive(
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier halfSpeedSup,
      BooleanSupplier angleLockSup) {
    return new DefaultDriveCmd(
        swerve,
        translationSup,
        strafeSup,
        rotationSup,
        halfSpeedSup,
        angleLockSup,
        this::getCurrentHeading,
        this::setCurrentHeading,
        this::getRotationLastTriggered,
        this::setRotationLastTriggered,
        this::getZoneLockedHeading);
  }

  public Command shootDrive(DoubleSupplier translationSup, DoubleSupplier strafeSup) {
    return new ShootDriveCmd(swerve, translationSup, strafeSup, this::setCurrentHeading);
  }

  public Command aimAtTargetPose() {
    return new AimAtTargetPoseCmd(swerve, this::setCurrentHeading, this::getCurrentAimTarget);
  }

  public Command swerveBrakeCmd() {
    return swerve.applyRequest(() -> brake);
  }

  public Command seedFieldCentricCmd() {
    return swerve.runOnce(swerve::seedFieldCentric);
  }

  /**
   * Operator-triggered vision reseed command. Unconditionally snaps swerve
   * odometry to the best
   * available multi-tag vision fix regardless of drift magnitude. Fires once on
   * button press
   * (InstantCommand). No-op if no vision fix is available.
   */
  public Command forceReseedFromVisionCmd() {
    return new InstantCommand(
        () -> {
          if (vision != null) {
            vision.forceReseedFromVision();
          }
        });
  }

  public void setDesiredSuperState(SuperState state) {
    this.state = state;
  }

  /**
   * Returns a command that transitions relevant subsystems to the requested
   * SuperState. Each
   * subsystem is controlled through proper WPILib command requirements — not
   * direct calls from
   * periodic().
   *
   * <p>
   * When the returned command ends (button released), the scheduler resumes the
   * default commands
   * on each required subsystem automatically.
   *
   * @param desiredState The SuperState to transition to
   * @return A command that controls subsystems for the duration of the state
   */
  public Command setStateCmd(SuperState desiredState) {
    return switch (desiredState) {
      case DRIVE_STARTING_CONFIG -> driveStartingConfigCmd();
      case DRIVE -> driveCmd();
      case INTAKE -> intakeCmd();
      case OUTTAKE -> outtakeCmd();
      case SHOOT_WITH_AIM -> shootWithAimCmd();
      case SHOOT_NO_AIM -> shootNoAimCmd();
      case MANUAL_SHOOT -> manualShootCmd();
      case PURGE -> purgeCmd();
    };
  }

  private Command driveStartingConfigCmd() {
    return Commands.run(
        () -> {
          setDesiredSuperState(SuperState.DRIVE_STARTING_CONFIG);
          intake.setDesiredState(IntakeState.HOME);
          hopper.setDesiredState(HopperState.STOP);
          shooter.setDesiredState(ShooterState.PREPFUEL);
        },
        intake,
        hopper,
        shooter)
        .withName("SuperState(DRIVE_STARTING_CONFIG)");
  }

  private Command driveCmd() {
    return Commands.run(
        () -> {
          setDesiredSuperState(SuperState.DRIVE);
          intake.setDesiredState(IntakeState.DEPLOYED);
          hopper.setDesiredState(HopperState.STOP);
          shooter.setDesiredState(ShooterState.PREPFUEL);
        },
        intake,
        hopper,
        shooter)
        .withName("SuperState(DRIVE)");
  }

  private Command intakeCmd() {
    return Commands.run(
        () -> {
          setDesiredSuperState(SuperState.INTAKE);
          intake.setDesiredState(IntakeState.INTAKE);
          hopper.setDesiredState(HopperState.STOP);
          shooter.setDesiredState(ShooterState.PREPFUEL);
        },
        intake,
        hopper,
        shooter)
        .withName("SuperState(INTAKE)");
  }

  private Command outtakeCmd() {
    return Commands.run(
        () -> {
          setDesiredSuperState(SuperState.OUTTAKE);
          intake.setDesiredState(IntakeState.OUTTAKE);
          hopper.setDesiredState(HopperState.INDEXTOINTAKE);
          shooter.setDesiredState(ShooterState.PREPFUEL);
        },
        intake,
        hopper,
        shooter)
        .withName("SuperState(OUTTAKE)");
  }

  private Command shootWithAimCmd() {
    if (!isShootAllowedZone()) {
      return Commands.idle().withName("SuperState(SHOOT_WITH_AIM:DISALLOWED)");
    }
    if (isPurgeZone()) {
      return createPurgeStateCommand().withName("SuperState(SHOOT_WITH_AIM->PURGE)");
    }
    return createShootStateCommand(true).withName("SuperState(SHOOT_WITH_AIM)");
  }

  private Command shootNoAimCmd() {
    if (!isShootAllowedZone()) {
      return Commands.idle().withName("SuperState(SHOOT_NO_AIM:DISALLOWED)");
    }
    return createShootStateCommand(false).withName("SuperState(SHOOT_NO_AIM)");
  }

  private Command manualShootCmd() {
    return Commands.run(
        () -> {
          setDesiredSuperState(SuperState.MANUAL_SHOOT);
          shooter.setDesiredState(ShooterState.SHOOT);
          setHopperFeedWhenShooterReady(false);
        },
        shooter,
        hopper)
        .alongWith(Commands.run(() -> swerve.setControl(brake), swerve))
        .withName("SuperState(MANUAL_SHOOT)");
  }

  private Command purgeCmd() {
    if (!isPurgeZone()) {
      return Commands.idle().withName("SuperState(PURGE:DISALLOWED)");
    }
    return createPurgeStateCommand().withName("SuperState(PURGE)");
  }

  /**
   * Override intake independently — preempts default, doesn't touch
   * hopper/shooter.
   */
  public Command intakeOverrideCmd(IntakeState intakeState) {
    return Commands.run(() -> intake.setDesiredState(intakeState), intake)
        .withName("IntakeOverride(" + intakeState.name() + ")");
  }

  /** Override hopper independently. */
  public Command hopperOverrideCmd(HopperState hopperState) {
    return Commands.runOnce(() -> hopper.setDesiredState(hopperState), hopper)
        .withName("HopperOverride(" + hopperState.name() + ")");
  }

  /** Override shooter independently. */
  public Command shooterOverrideCmd(ShooterState shooterState) {
    return Commands.runOnce(() -> shooter.setDesiredState(shooterState), shooter)
        .withName("ShooterOverride(" + shooterState.name() + ")");
  }

  public ShootMode getShootMode() {
    return shootMode;
  }

  public void setShootMode(ShootMode shootMode) {
    this.shootMode = shootMode;
    DogLog.log("Superstructure/ShootMode", shootMode.name());
  }

  public Command setShootModeCmd(ShootMode shootMode) {
    return Commands.runOnce(() -> setShootMode(shootMode));
  }

  public Command toggleShootModeCmd(ShootMode manualMode) {
    return Commands.runOnce(
        () -> {
          ShootMode nextMode = shootMode == manualMode ? ShootMode.DEFAULT_SHOOT_WITH_AIM : manualMode;
          setShootMode(nextMode);
        });
  }

  /* Alignment */

  /**
   * Returns true if the robot heading is within tolerance of the angle to the
   * hub.
   */
  public boolean isAlignedToTarget() {
    return alignedToTarget;
  }

  static double resolveGeometricTargetHeadingRadians(Pose2d currentPose, Translation2d target) {
    double dx = target.getX() - currentPose.getX();
    double dy = target.getY() - currentPose.getY();
    return Math.atan2(dy, dx);
  }

  static double resolveOperatorPerspectiveTargetHeadingRadians(
      Pose2d currentPose, Translation2d hubTarget, DriverStation.Alliance alliance) {
    double dx = hubTarget.getX() - currentPose.getX();
    double dy = hubTarget.getY() - currentPose.getY();
    double targetAngleRad = Math.atan2(dy, dx);

    if (alliance == DriverStation.Alliance.Red) {
      targetAngleRad += Math.PI;
      targetAngleRad = Math.IEEEremainder(targetAngleRad, 2.0 * Math.PI);
    }

    return targetAngleRad;
  }

  static boolean isHeadingAlignedToTarget(
      Pose2d currentPose, Translation2d target, double toleranceDegrees) {
    double targetAngleRad = resolveGeometricTargetHeadingRadians(currentPose, target);

    double headingErrorRad = currentPose.getRotation().getRadians() - targetAngleRad;
    headingErrorRad = Math.IEEEremainder(headingErrorRad, 2.0 * Math.PI);

    return Math.abs(Math.toDegrees(headingErrorRad)) < toleranceDegrees;
  }

  /** Zero-allocation alignment check using raw atan2 math. */
  private void updateAlignmentStatus(Pose2d currentPose, Translation2d target) {
    alignedToTarget = isHeadingAlignedToTarget(currentPose, target, ALIGNMENT_TOLERANCE_DEGREES);
  }

  /**
   * Returns the desired locked heading angle (in degrees) based on the current
   * field zone and
   * alliance. Returns empty if no lock is defined for the zone.
   */
  public Optional<Rotation2d> getZoneLockedHeading() {
    if (!allianceConfirmed || currentZone == null)
      return Optional.empty();

    double leftLockDegrees = alliance == DriverStation.Alliance.Red ? 135.0 : -45.0;
    double rightLockDegrees = alliance == DriverStation.Alliance.Red ? -135.0 : 45.0;

    return switch (currentZone) {
      case ALLIANCE_LEFT,
          NEUTRAL_LEFT_SHOOT,
          NEUTRAL_LEFT_PURGE,
          NEUTRAL_LEFT,
          OPPONENT_LEFT ->
        Optional.of(Rotation2d.fromDegrees(leftLockDegrees));
      case ALLIANCE_RIGHT,
          NEUTRAL_RIGHT_SHOOT,
          NEUTRAL_RIGHT_PURGE,
          NEUTRAL_RIGHT,
          OPPONENT_RIGHT ->
        Optional.of(Rotation2d.fromDegrees(rightLockDegrees));
      default -> Optional.empty();
    };
  }

  /* Hub shift accessors */

  /**
   * Returns true if our hub is currently active and we should be shooting. Uses
   * the shifted
   * (fudged) timing so fuel arrives within the active window.
   *
   * <p>
   * During auto, always returns true (hub is always active for your alliance).
   * During disabled,
   * returns false.
   */
  public boolean isHubActive() {
    return HubShiftUtil.getShiftedShiftInfo().active();
  }

  /**
   * Returns the time remaining in the current shift (using shifted timing).
   * Drivers can use this to
   * decide whether to commit to a scoring cycle or reposition for the next active
   * shift.
   */
  public double getShiftTimeRemaining() {
    return HubShiftUtil.getShiftedShiftInfo().remainingTime();
  }

  /** Updates cached targeting state and publishes superstructure telemetry. */
  @Override
  public void periodic() {
    DogLog.time("Perf/Superstructure");

    refreshAllianceAndCachedHubTarget();
    Pose2d currentPose = swerve.getState().Pose;

    updateCurrentZone(currentPose);
    updateShooterSetpointAndAlignment(currentPose);
    updateVisionAimingMode();
    logHubShiftTelemetry();
    logSuperstructureTelemetry();

    DogLog.timeEnd("Perf/Superstructure");
  }

  private void updateCurrentZone(Pose2d currentPose) {
    if (allianceConfirmed) {
      currentZone = FieldZones.fromPose(currentPose, alliance);
      DogLog.log("Superstructure/Zone", ZONE_NAMES[currentZone.ordinal()]);
    }
  }

  private void updateShooterSetpointAndAlignment(Pose2d currentPose) {
    Translation2d aimTarget = getCurrentAimTarget();
    if (aimTarget != null) {
      double distanceToTarget = currentPose.getTranslation().getDistance(aimTarget);
      DogLog.log("Superstructure/Distance to Target (feet)", Units.metersToFeet(distanceToTarget));
      updateShooterSetpointForDistance(distanceToTarget);
      updateAlignmentStatus(currentPose, aimTarget);
    }
  }

  private void updateShooterSetpointForDistance(double distanceToTarget) {
    if (state == SuperState.MANUAL_SHOOT) {
      return;
    }

    shooter.setSetpointForDistance(distanceToTarget);
    if (isNeutralShootOrPurgeZone()) {
      shooter.setSetpoint(shooter.getTargetRPM(), NEUTRAL_ZONE_HOOD_LOCK);
    }
  }

  private void updateVisionAimingMode() {
    if (vision != null) {
      vision.setAiming(state == SuperState.SHOOT_WITH_AIM || state == SuperState.SHOOT_NO_AIM);
    }
  }

  private void logHubShiftTelemetry() {
    ShiftInfo officialShift = HubShiftUtil.getOfficialShiftInfo();
    ShiftInfo shiftedShift = HubShiftUtil.getShiftedShiftInfo();

    logShiftInfo("HubShift/Official", officialShift, true);
    logShiftInfo("HubShift/Shifted", shiftedShift, false);
    DogLog.log("HubShift/FirstActiveAlliance", HubShiftUtil.getFirstActiveAlliance().name());
  }

  private static void logShiftInfo(String prefix, ShiftInfo shiftInfo, boolean publishToNt) {
    if (publishToNt) {
      DogLog.forceNt.log(prefix + "/CurrentShift", shiftInfo.currentShift().name());
      DogLog.forceNt.log(prefix + "/Active", shiftInfo.active());
      DogLog.log(prefix + "/ElapsedTime", shiftInfo.elapsedTime());
      DogLog.forceNt.log(prefix + "/RemainingTime", shiftInfo.remainingTime());
      return;
    }

    DogLog.log(prefix + "/CurrentShift", shiftInfo.currentShift().name());
    DogLog.log(prefix + "/Active", shiftInfo.active());
    DogLog.log(prefix + "/ElapsedTime", shiftInfo.elapsedTime());
    DogLog.log(prefix + "/RemainingTime", shiftInfo.remainingTime());
  }

  private void logSuperstructureTelemetry() {
    DogLog.forceNt.log("Superstructure/CurrentState", state.toString());
    DogLog.forceNt.log("Superstructure/IsAlignedToTarget", alignedToTarget);
    DogLog.forceNt.log(
        "Superstructure/ShootMode/DefaultShootWithAim",
        shootMode == ShootMode.DEFAULT_SHOOT_WITH_AIM);
    DogLog.forceNt.log(
        "Superstructure/ShootMode/ManualBumperUp", shootMode == ShootMode.MANUAL_BUMPER_UP);
    DogLog.forceNt.log(
        "Superstructure/ShootMode/ManualTrench", shootMode == ShootMode.MANUAL_TRENCH);
  }
}
