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
    SHOOT,
    MANUAL_SHOOT,
    // PURGE
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
  private Translation2d cachedHubTarget;
  private boolean allianceConfirmed = false;
  private FieldZones currentZone;
  private boolean manualShooterDistanceOverride = false;

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
    cachedHubTarget = FieldConstants.Hub.BLUE_CENTER_POSE;
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
    cachedHubTarget = alliance == DriverStation.Alliance.Red
        ? FieldConstants.Hub.RED_CENTER_POSE
        : FieldConstants.Hub.BLUE_CENTER_POSE;
  }

  private void refreshAllianceAndCachedHubTarget() {
    DriverStation.getAlliance()
        .ifPresent(
            dsAlliance -> {
              setAlliance(dsAlliance);
              allianceConfirmed = true;
              DogLog.log("Robot/AllianceConfirmed", dsAlliance.name());
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
    return new AimAtTargetPoseCmd(swerve, this::setCurrentHeading);
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
    DogLog.log("Superstructure/State", state.name());
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
      case DRIVE_STARTING_CONFIG:
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
      case DRIVE:
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

      case INTAKE:
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

      case OUTTAKE:
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

      case SHOOT:
        // SHOOT is special — it needs continuous polling for alignment.
        // swerve is always required: when override is on, hold brake immediately;
        // otherwise actively aim until the synchronous alignment flag is true,
        // then switch to brake (X-lock) and hold that for the rest of the trigger
        // hold. This preserves deterministic handoff without relying on the CTRE
        // async heading controller state.
        return Commands.run(
            () -> {
              setDesiredSuperState(SuperState.SHOOT);
              shooter.setDesiredState(ShooterState.SHOOT);
              if (manualShooterDistanceOverride) {
                // Override active: brake held by parallel command; just gate hopper
                if (shooter.getCurrentState() == ShooterState.SHOOT) {
                  hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
                } else {
                  hopper.setDesiredState(HopperState.STOP);
                }
              } else {
                // Normal: require alignment before feeding;
                // AimAtTargetPoseCmd handles swerve in parallel
                if (shooter.getCurrentState() == ShooterState.SHOOT && isAlignedToTarget()) {
                  hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
                } else {
                  hopper.setDesiredState(HopperState.STOP);
                }
              }
            },
            shooter,
            hopper)
            .alongWith(
                Commands.either(
                    new AimAtTargetPoseCmd(swerve, this::setCurrentHeading)
                        .until(this::isAlignedToTarget)
                        .andThen(Commands.run(() -> swerve.setControl(brake), swerve)),
                    Commands.run(() -> swerve.setControl(brake), swerve),
                    () -> !manualShooterDistanceOverride))
            .withName("SuperState(SHOOT)");
      case MANUAL_SHOOT:
        // SHOOT is special — it needs continuous polling for alignment.
        // swerve is always required: when override is on, hold brake immediately;
        // otherwise aim until AimAtTargetPoseCmd finishes on target, then hold
        // brake so the default drive command cannot resume and fight the shot.
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
      // case PURGE:
      // // SHOOT is special — it needs continuous polling for alignment.
      // // swerve is always required: when override is on, hold brake immediately;
      // // otherwise aim until AimAtTargetPoseCmd finishes on target, then hold
      // // brake so the default drive command cannot resume and fight the shot.
      // return Commands.run(
      // () -> {
      // setDesiredSuperState(SuperState.PURGE);
      // shooter.setDesiredState(ShooterState.PURGE);
      // if (shooter.getCurrentState() == ShooterState.PURGE) {
      // hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
      // intake.setDesiredState(IntakeState.OUTTAKE);
      // } else {
      // hopper.setDesiredState(HopperState.STOP);
      // intake.setDesiredState(IntakeState.DEPLOYED);
      // }
      // },
      // shooter,
      // hopper,
      // intake)
      // .alongWith(Commands.run(() -> swerve.setControl(brake), swerve))
      // .withName("SuperState(PURGE)");
      default:
        return Commands.none();
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

  public Command enableManualShooterDistanceOverrideCmd() {
    // When this is enabled, only do bumper up shot settings. Ignore all calculated
    // values
    return Commands.runOnce(
        () -> {
          manualShooterDistanceOverride = true;
          shooter.setManualDistanceOverride(manualShooterDistanceOverride);
        },
        shooter)
        .withName("ShooterOverride(Manual Distance Override Enable)");
  }

  public Command disableManualShooterDistanceOverrideCmd() {
    // When this is disabled, revert to calculated shot settings.
    return Commands.runOnce(
        () -> {
          manualShooterDistanceOverride = false;
          shooter.setManualDistanceOverride(manualShooterDistanceOverride);
        },
        shooter)
        .withName("ShooterOverride(Manual Distance Override Disable)");
  }

  public Command toggleManualShooterDistanceOverrideCmd() {
    // Each press flips the override state: enabled → disabled → enabled...
    return Commands.runOnce(
        () -> {
          manualShooterDistanceOverride = !manualShooterDistanceOverride;
          shooter.setManualDistanceOverride(manualShooterDistanceOverride);
          DogLog.log("Shooter/ManualDistanceOverride", manualShooterDistanceOverride);
        },
        shooter)
        .withName("ShooterOverride(Toggle Manual Distance Override)");
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

    switch (currentZone) {
      case ALLIANCE_LEFT, NEUTRAL_LEFT, OPPONENT_LEFT:
        return Optional.of(Rotation2d.fromDegrees(leftLockDegrees));
      case ALLIANCE_RIGHT, NEUTRAL_RIGHT, OPPONENT_RIGHT:
        return Optional.of(Rotation2d.fromDegrees(rightLockDegrees));
      default:
        return Optional.empty();
    }
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
      DogLog.log("Robot/Zone", ZONE_NAMES[currentZone.ordinal()]);
    }

    // ── Distance + alignment (needed by SHOOT command group) ──────────────
    if (cachedHubTarget != null) {
      double distanceToHub = currentPose.getTranslation().getDistance(cachedHubTarget);
      DogLog.log("Superstructure/Distance to Hub (feet)", Units.metersToFeet(distanceToHub));
      shooter.setSetpointForDistance(distanceToHub);
      updateAlignmentStatus(currentPose, cachedHubTarget);
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

    DogLog.log("SuperStructure/SuperStructure_CurrentState", state.toString());
    DogLog.log("SuperStructure/SuperStructure_IsAlignedToTarget", alignedToTarget);

    DogLog.timeEnd("Perf/Superstructure");
  }
}
