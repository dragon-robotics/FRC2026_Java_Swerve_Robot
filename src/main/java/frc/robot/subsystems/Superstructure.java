package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.util.Telemetry;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.ZONES;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Superstructure extends SubsystemBase {

  // ──────────────────────────────────────────────────────────────────────────
  // State Enum
  // ──────────────────────────────────────────────────────────────────────────

  public enum SuperState {
    DRIVE,
    INTAKE,
    OUTTAKE,
    SHOOT
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

  private static final double ALIGNMENT_TOLERANCE_DEGREES = 3.0;
  private boolean alignedToTarget = false;
  private Translation2d cachedHubTarget;

  // ──────────────────────────────────────────────────────────────────────────
  // State Machine
  // ──────────────────────────────────────────────────────────────────────────

  private SuperState state;
  private SuperState lastState = null;

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

    state = SuperState.DRIVE;

    brake = new SwerveRequest.SwerveDriveBrake();
    point = new SwerveRequest.PointWheelsAt();
    applyFieldSpeeds =
        new SwerveRequest.ApplyFieldSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withDriveRequestType(DriveRequestType.Velocity);
    applyRobotSpeeds =
        new SwerveRequest.ApplyRobotSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withDriveRequestType(DriveRequestType.Velocity);

    currentHeading = Optional.empty();
    rotationLastTriggered = 0.0;

    logger = new Telemetry();
    swerve.registerTelemetry(logger::telemeterize);

    /* Always assume blue alliance if no alliance is found */
    alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    cachedHubTarget =
        alliance == DriverStation.Alliance.Red
            ? FieldConstants.Hub.RED_CENTER_POSE
            : FieldConstants.Hub.BLUE_CENTER_POSE;
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
      BooleanSupplier halfSpeedSup) {
    return new DefaultDriveCmd(
        swerve,
        translationSup,
        strafeSup,
        rotationSup,
        halfSpeedSup,
        this::getCurrentHeading,
        this::setCurrentHeading,
        this::getRotationLastTriggered,
        this::setRotationLastTriggered);
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
   * Operator-triggered vision reseed command. Unconditionally snaps swerve odometry to the best
   * available multi-tag vision fix regardless of drift magnitude. Fires once on button press
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

  public Command intakeCommand() {
    return new InstantCommand(() -> intake.setDesiredState(IntakeState.INTAKE), intake);
  }

  public Command outtakeCommand() {
    return new InstantCommand(() -> intake.setDesiredState(IntakeState.OUTTAKE), intake);
  }

  public Command deployIntakeCommand() {
    return new InstantCommand(() -> intake.setDesiredState(IntakeState.DEPLOYED), intake);
  }

  public Command stowIntakeCommand() {
    return new InstantCommand(() -> intake.setDesiredState(IntakeState.HOME), intake);
  }

  public Command wokTossIntakeCommand() {
    return new InstantCommand(() -> intake.setDesiredState(IntakeState.WOKTOSS), intake);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Hopper Commands
  // ──────────────────────────────────────────────────────────────────────────

  public Command indexToIntakeCommand() {
    return new InstantCommand(() -> hopper.setDesiredState(HopperState.INDEXTOINTAKE), hopper);
  }

  public Command indexToShooterCommand() {
    return new InstantCommand(() -> hopper.setDesiredState(HopperState.INDEXTOSHOOTER), hopper);
  }

  public Command stopHopperCommand() {
    return new InstantCommand(() -> hopper.setDesiredState(HopperState.STOP), hopper);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Shooter Commands
  // ──────────────────────────────────────────────────────────────────────────

  public Command stopShooterCommand() {
    return new InstantCommand(() -> shooter.setDesiredState(ShooterState.STOP), shooter);
  }

  public Command shootCommand() {
    return new InstantCommand(() -> shooter.setDesiredState(ShooterState.SHOOT), shooter);
  }

  public Command prepFuelCommand() {
    return new InstantCommand(() -> shooter.setDesiredState(ShooterState.PREPFUEL), shooter);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // SuperState Commands
  // ──────────────────────────────────────────────────────────────────────────

  public Command driveSuperstateCommand() {
    return new InstantCommand(() -> setDesiredSuperState(SuperState.DRIVE));
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Alignment
  // ──────────────────────────────────────────────────────────────────────────

  /** Returns true if the robot heading is within tolerance of the angle to the hub. */
  public boolean isAlignedToTarget() {
    return alignedToTarget;
  }

  /** Zero-allocation alignment check using raw atan2 math. */
  private void updateAlignmentStatus(Pose2d currentPose, Translation2d hubTarget) {
    double dx = hubTarget.getX() - currentPose.getX();
    double dy = hubTarget.getY() - currentPose.getY();
    double targetAngleRad = Math.atan2(dy, dx);

    double headingErrorRad = currentPose.getRotation().getRadians() - targetAngleRad;
    headingErrorRad = Math.IEEEremainder(headingErrorRad, 2.0 * Math.PI);

    alignedToTarget = Math.abs(Math.toDegrees(headingErrorRad)) < ALIGNMENT_TOLERANCE_DEGREES;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // State Machine
  // ──────────────────────────────────────────────────────────────────────────

  public void setDesiredSuperState(SuperState state) {
    this.state = state;
  }

  public void handleStateTransition() {
    switch (state) {
      case DRIVE:
        intake.setDesiredState(IntakeState.DEPLOYED);
        hopper.setDesiredState(HopperState.STOP);
        shooter.setDesiredState(ShooterState.PREPFUEL);
        break;

      case INTAKE:
        intake.setDesiredState(IntakeState.INTAKE);
        shooter.setDesiredState(ShooterState.STOP);
        break;

      case OUTTAKE:
        intake.setDesiredState(IntakeState.OUTTAKE);
        hopper.setDesiredState(HopperState.INDEXTOINTAKE);
        shooter.setDesiredState(ShooterState.STOP);
        break;

      case SHOOT:
        shooter.setDesiredState(ShooterState.SHOOT);
        if (shooter.getCurrentState() == ShooterState.SHOOT && isAlignedToTarget()) {
          // Target acquired — X-lock wheels and feed the ball
          swerve.setControl(brake);
          hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
        } else {
          hopper.setDesiredState(HopperState.STOP);
        }
        break;
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Periodic
  // ──────────────────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    DogLog.time("Perf/Superstructure");

    // Compute distance and alignment
    Pose2d currentPose = swerve.getState().Pose;

    // Check alliance zone the robot is in
    ZONES currentZone = ZONES.fromPose(currentPose, alliance);
    DogLog.log("Robot/Zone", currentZone.name());

    double distanceToHub = currentPose.getTranslation().getDistance(cachedHubTarget);
    DogLog.log("Superstructure/Distance to Hub (feet)", Units.metersToFeet(distanceToHub));

    shooter.setSetpointForDistance(
        distanceToHub + Units.feetToMeters(0.5)); // Always add 0.5 feet to account for
    // shooter rpm drop
    updateAlignmentStatus(currentPose, cachedHubTarget);

    // Reseed swerve pose from vision if odometry has drifted significantly.
    // vision.tryReseedFromVision() is a no-op when no qualifying vision fix is
    // available this cycle, so this is safe to call unconditionally.
    if (vision != null) {
      vision.tryReseedFromVision(currentPose);
    }

    handleStateTransition();

    DogLog.timeEnd("Perf/Superstructure");
  }
}
