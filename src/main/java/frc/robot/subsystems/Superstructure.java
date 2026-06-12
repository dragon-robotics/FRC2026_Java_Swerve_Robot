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

public class Superstructure extends SubsystemBase {

  // ──────────────────────────────────────────────────────────────────────────
  // State Enum
  // ──────────────────────────────────────────────────────────────────────────

  public enum SuperState {
    DRIVE_STARTING_CONFIG,
    DRIVE,
    INTAKE,
    OUTTAKE,
    SHOOT_WITH_AIM, // Shooting while aiming with vision assistance (default shoot behavior)
    SHOOT_NO_AIM, // Shooting without aiming assistance, but still calculating shooter setpoint
    // based on distance using vision pose estimation (e.g., bumper up shot)
    MANUAL_SHOOT, // Shooting with manual setpoint and no vision assistance if vision is bad or
    // unavailable
    PURGE // Shooting while outtaking to get rid of FUEL as fast as possible for passing
    // purposes
  }

  // Used to select which shooting state we are in
  public enum ShootMode {
    DEFAULT_SHOOT_WITH_AIM,
    MANUAL_BUMPER_UP,
    MANUAL_TRENCH
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Subsystem References
  // ──────────────────────────────────────────────────────────────────────────

  private final CommandSwerveDrivetrain swerve;
  private final IntakeSubsystem intake;
  private final HopperSubsystem hopper;
  private final ShooterSubsystem shooter;
  private final VisionSubsystem vision;
  private final RobotContainer container;
  private final Telemetry logger;

  // ──────────────────────────────────────────────────────────────────────────
  // Swerve Requests
  // ──────────────────────────────────────────────────────────────────────────

  private final SwerveRequest.SwerveDriveBrake brake;
  private final SwerveRequest.PointWheelsAt point;
  private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds;
  private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds;

  // ──────────────────────────────────────────────────────────────────────────
  // Heading Tracking
  // ──────────────────────────────────────────────────────────────────────────

  private Optional<Rotation2d> currentHeading;
  private double rotationLastTriggered;

  // ──────────────────────────────────────────────────────────────────────────
  // Alignment & Targeting
  // ──────────────────────────────────────────────────────────────────────────

  private static final double ALIGNMENT_TOLERANCE_DEGREES = 5;
  private boolean alignedToTarget = false;
  private boolean allianceConfirmed = false;
  private FieldZones currentZone;
  private ShootMode shootMode = ShootMode.DEFAULT_SHOOT_WITH_AIM;

  // Used for manual override if vision fails and we need to shoot with driver
  // manually aiming
  private static final double MANUAL_BUMPER_UP_RPM = 2500.0;
  private static final double MANUAL_BUMPER_UP_HOOD = 0.0;
  private static final double MANUAL_TRENCH_RPM = 2900.0;
  private static final double MANUAL_TRENCH_HOOD = 0.75;
  private static final double NEUTRAL_ZONE_HOOD_LOCK = 2.0;

  // Pre-cached zone name strings — avoids .name() heap allocation every cycle
  private static final String[] ZONE_NAMES;

  static {
    FieldZones[] zones = FieldZones.values();
    ZONE_NAMES = new String[zones.length];
    for (int i = 0; i < zones.length; i++) {
      ZONE_NAMES[i] = zones[i].name();
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // State Machine
  // ──────────────────────────────────────────────────────────────────────────

  private SuperState state;

  // ──────────────────────────────────────────────────────────────────────────
  // Alliance Initialization
  // ──────────────────────────────────────────────────────────────────────────
  private DriverStation.Alliance alliance;

  // ──────────────────────────────────────────────────────────────────────────
  // Constructor
  // ──────────────────────────────────────────────────────────────────────────

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

    // ── Default Commands ─────────────────────────────────────────────────
    // These run whenever no other command requires the subsystem.
    // They define the "DRIVE" superstate behavior. When any button-triggered
    // command ends, the scheduler automatically resumes these defaults —
    // no god loop needed.
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

  private Translation2d getHubTargetForAlliance() {
    return alliance == DriverStation.Alliance.Red
        ? FieldConstants.Hub.RED_CENTER_POSE
        : FieldConstants.Hub.BLUE_CENTER_POSE;
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
      default ->
        isRed ? FieldConstants.Hub.RED_CENTER_POSE : FieldConstants.Hub.BLUE_CENTER_POSE;
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
    return Commands.run(
        () -> {
          setDesiredSuperState(withAim ? SuperState.SHOOT_WITH_AIM : SuperState.SHOOT_NO_AIM);
          shooter.setDesiredState(ShooterState.SHOOT);
          // Require alignment before feeding; AimAtTargetPoseCmd handles swerve in
          // parallel.
          if (shooter.getCurrentState() == ShooterState.SHOOT && isAlignedToTarget()) {
            hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
          } else {
            hopper.setDesiredState(HopperState.STOP);
          }
        },
        shooter,
        hopper)
        .alongWith(
            new AimAtTargetPoseCmd(swerve, this::setCurrentHeading, this::getCurrentAimTarget)
                .until(this::isAlignedToTarget)
                .andThen(Commands.run(() -> swerve.setControl(brake), swerve)));
  }

  private Command createPurgeStateCommand() {
    return Commands.run(
        () -> {
          setDesiredSuperState(SuperState.PURGE);
          intake.setDesiredState(IntakeState.OUTTAKE);
          shooter.setDesiredState(ShooterState.SHOOT);
          if (shooter.getCurrentState() == ShooterState.SHOOT && isAlignedToTarget()) {
            hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
          } else {
            hopper.setDesiredState(HopperState.STOP);
          }
        },
        intake,
        shooter,
        hopper)
        .alongWith(
            new AimAtTargetPoseCmd(swerve, this::setCurrentHeading, this::getCurrentAimTarget)
                .until(this::isAlignedToTarget)
                .andThen(Commands.run(() -> swerve.setControl(brake), swerve)));
  }

  private Command createManualShootStateCommand(double shooterRpm, double hoodAngle) {
    return Commands.run(
        () -> {
          shooter.setSetpoint(shooterRpm, hoodAngle);
          setDesiredSuperState(SuperState.MANUAL_SHOOT);
          shooter.setDesiredState(ShooterState.SHOOT);
          if (shooter.getCurrentState() == ShooterState.SHOOT) {
            hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
          } else {
            hopper.setDesiredState(HopperState.STOP);
          }
        },
        shooter,
        hopper)
        .alongWith(Commands.run(() -> swerve.setControl(brake), swerve))
        .withName("SuperState(MANUAL_SHOOT)");
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
                  if (shooter.getCurrentState() == ShooterState.SHOOT && isAlignedToTarget()) {
                    hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
                  } else {
                    hopper.setDesiredState(HopperState.STOP);
                  }
                },
                intake,
                shooter,
                hopper)
                .alongWith(
                    new AimAtTargetPoseCmd(
                        swerve, this::setCurrentHeading, this::getCurrentAimTarget)
                        .until(this::isAlignedToTarget)
                        .andThen(Commands.run(() -> swerve.setControl(brake), swerve))))
        .withName("SuperState(SHOOT_WITH_AIM+Juicer)");
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
              setAlliance(dsAlliance);
              allianceConfirmed = true;
              DogLog.log("Superstructure/AllianceConfirmed", dsAlliance.name());
            });
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Heading Accessors (used by DefaultDriveCmd)
  // ──────────────────────────────────────────────────────────────────────────

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

  // ──────────────────────────────────────────────────────────────────────────
  // Drive Commands
  // ──────────────────────────────────────────────────────────────────────────

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

  // ──────────────────────────────────────────────────────────────────────────
  // Intake Commands
  // ──────────────────────────────────────────────────────────────────────────

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
    switch (desiredState) {
      case DRIVE_STARTING_CONFIG -> {
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
      case DRIVE -> {
        // No subsystem commands needed — releasing any other state command
        // causes the scheduler to resume default commands, which already
        // implement DRIVE behavior (intake=DEPLOYED, hopper=STOP, shooter=PREPFUEL).
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

      case INTAKE -> {
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

      case OUTTAKE -> {
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

      case SHOOT_WITH_AIM -> {
        // SHOOT_WITH_AIM is only enabled in designated field zones.
        if (!isShootAllowedZone()) {
          return Commands.idle().withName("SuperState(SHOOT_WITH_AIM:DISALLOWED)");
        }
        if (isPurgeZone()) {
          return createPurgeStateCommand().withName("SuperState(SHOOT_WITH_AIM->PURGE)");
        }
        return createShootStateCommand(true).withName("SuperState(SHOOT_WITH_AIM)");
      }
      case SHOOT_NO_AIM -> {
        // SHOOT_NO_AIM is only enabled in designated field zones.
        if (!isShootAllowedZone()) {
          return Commands.idle().withName("SuperState(SHOOT_NO_AIM:DISALLOWED)");
        }
        return createShootStateCommand(false).withName("SuperState(SHOOT_NO_AIM)");
      }
      case MANUAL_SHOOT -> {
        return Commands.run(
            () -> {
              setDesiredSuperState(SuperState.MANUAL_SHOOT);
              shooter.setDesiredState(ShooterState.SHOOT);
              if (shooter.getCurrentState() == ShooterState.SHOOT) {
                hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
              } else {
                hopper.setDesiredState(HopperState.STOP);
              }
            },
            shooter,
            hopper)
            .alongWith(Commands.run(() -> swerve.setControl(brake), swerve))
            .withName("SuperState(MANUAL_SHOOT)");
      }
      case PURGE -> {
        if (!isPurgeZone()) {
          return Commands.idle().withName("SuperState(PURGE:DISALLOWED)");
        }
        return createPurgeStateCommand().withName("SuperState(PURGE)");
      }
      default -> {
        return Commands.none();
      }
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Individual Subsystem Override Commands
  //
  // These require ONLY their specific subsystem — they preempt the default
  // command on that subsystem without affecting others. When released,
  // the default command resumes automatically.
  // ──────────────────────────────────────────────────────────────────────────

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

  // ──────────────────────────────────────────────────────────────────────────
  // Alignment
  // ──────────────────────────────────────────────────────────────────────────

  /**
   * Returns true if the robot heading is within tolerance of the angle to the
   * hub.
   */
  public boolean isAlignedToTarget() {
    return alignedToTarget;
  }

  /** Zero-allocation alignment check using raw atan2 math. */
  private void updateAlignmentStatus(Pose2d currentPose, Translation2d hubTarget) {
    double dx = hubTarget.getX() - currentPose.getX();
    double dy = hubTarget.getY() - currentPose.getY();
    double targetAngleRad = Math.atan2(dy, dx);

    // Match AimAtTargetPoseCmd heading convention exactly so hopper feed gating
    // and aim completion use the same notion of "aligned".
    if (alliance == DriverStation.Alliance.Red) {
      targetAngleRad += Math.PI;
      targetAngleRad = Math.IEEEremainder(targetAngleRad, 2.0 * Math.PI);
    }

    double headingErrorRad = currentPose.getRotation().getRadians() - targetAngleRad;
    headingErrorRad = Math.IEEEremainder(headingErrorRad, 2.0 * Math.PI);

    alignedToTarget = Math.abs(Math.toDegrees(headingErrorRad)) < ALIGNMENT_TOLERANCE_DEGREES;
  }

  // Heading Lock based on zone based on user input
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

  // ──────────────────────────────────────────────────────────────────────────
  // Hub Shift Accessors
  // ──────────────────────────────────────────────────────────────────────────

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

  // ──────────────────────────────────────────────────────────────────────────
  // Periodic — telemetry and vision reseed ONLY, no subsystem state writes
  // ──────────────────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    DogLog.time("Perf/Superstructure");

    // ── Alliance (poll until confirmed, then never again) ──────────────────
    if (!allianceConfirmed) {
      refreshAllianceAndCachedHubTarget();
    }

    // Always compute telemetry regardless of alliance confirmation
    Pose2d currentPose = swerve.getState().Pose;

    // ── Zone detection (zero-alloc via pre-cached name strings) ────────────
    if (allianceConfirmed) {
      currentZone = FieldZones.fromPose(currentPose, alliance);
      DogLog.log("Superstructure/Zone", ZONE_NAMES[currentZone.ordinal()]);
    }

    // ── Distance + alignment (needed by SHOOT command group) ──────────────
    Translation2d aimTarget = getCurrentAimTarget();
    if (aimTarget != null) {
      double distanceToTarget = currentPose.getTranslation().getDistance(aimTarget);
      DogLog.log("Superstructure/Distance to Target (feet)", Units.metersToFeet(distanceToTarget));
      if (state != SuperState.MANUAL_SHOOT) {
        shooter.setSetpointForDistance(distanceToTarget);
        if (isNeutralShootOrPurgeZone()) {
          shooter.setSetpoint(shooter.getTargetRPM(), NEUTRAL_ZONE_HOOD_LOCK);
        }
      }
      updateAlignmentStatus(currentPose, aimTarget);
    }

    // // ── Vision reseed — no-op if no qualifying fix available ──────────────
    // if (vision != null) {
    // vision.tryReseedFromVision(currentPose);
    // }

    // ── Hub Shift Tracking ─────────────────────────────────────────────────
    ShiftInfo officialShift = HubShiftUtil.getOfficialShiftInfo();
    ShiftInfo shiftedShift = HubShiftUtil.getShiftedShiftInfo();

    // Official shift info (matches FMS timing exactly)
    DogLog.log("HubShift/Official/CurrentShift", officialShift.currentShift().name());
    DogLog.log("HubShift/Official/Active", officialShift.active());
    DogLog.log("HubShift/Official/ElapsedTime", officialShift.elapsedTime());
    DogLog.log("HubShift/Official/RemainingTime", officialShift.remainingTime());

    // Shifted info (accounts for fuel flight time + count delay).
    // Use THIS for robot decisions — it tells you when to START and STOP
    // shooting so fuel arrives during the active window.
    DogLog.log("HubShift/Shifted/CurrentShift", shiftedShift.currentShift().name());
    DogLog.log("HubShift/Shifted/Active", shiftedShift.active());
    DogLog.log("HubShift/Shifted/ElapsedTime", shiftedShift.elapsedTime());
    DogLog.log("HubShift/Shifted/RemainingTime", shiftedShift.remainingTime());

    // Which alliance goes first (for driver awareness)
    DogLog.log("HubShift/FirstActiveAlliance", HubShiftUtil.getFirstActiveAlliance().name());

    DogLog.log("Superstructure/CurrentState", state.toString());
    DogLog.log("Superstructure/IsAlignedToTarget", alignedToTarget);
    DogLog.log(
        "Superstructure/ShootMode/DefaultShootWithAim",
        shootMode == ShootMode.DEFAULT_SHOOT_WITH_AIM);
    DogLog.log("Superstructure/ShootMode/ManualBumperUp", shootMode == ShootMode.MANUAL_BUMPER_UP);
    DogLog.log("Superstructure/ShootMode/ManualTrench", shootMode == ShootMode.MANUAL_TRENCH);

    DogLog.timeEnd("Perf/Superstructure");
  }
}
