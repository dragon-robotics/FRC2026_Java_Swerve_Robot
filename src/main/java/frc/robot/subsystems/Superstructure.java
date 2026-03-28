package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
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
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Superstructure extends SubsystemBase {

  public enum SuperState {
    DRIVE, // DRIVE
    INTAKE, // DRIVE, SHOOTER PREPFUEL, HOPPER INDEXTOSHOOTER, INTAKE INTAKE
    OUTTAKE, // DRIVE, SHOOTER PREPFUEL, HOPPER INDEXTOINTAKE, INTAKE OUTTAKE
    SHOOT, // DRIVE points towards target, SHOOTER SHOOT, HOPPER INDEXTOSHOOTER, INTAKE OFF
    SHOOT_JUICER
  }

  private SuperState state;
  private SuperState lastState = null; // ← Add this line

  private final CommandSwerveDrivetrain swerve;
  private final IntakeSubsystem intake;
  private final HopperSubsystem hopper;
  private final ShooterSubsystem shooter;
  private final VisionSubsystem vision;
  private final RobotContainer container;
  private final Telemetry logger;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.SwerveDriveBrake brake;
  private final SwerveRequest.PointWheelsAt point;

  /* Used for path following */
  private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds;
  private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds;

  private Optional<Rotation2d> currentHeading; // Keeps track of current heading
  private double rotationLastTriggered; // Keeps track of the last time the rotation was triggered

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

    /* Initialize superstate */
    state = SuperState.DRIVE;

    /* Instantiate brake (X-lock swerve wheels) */
    brake = new SwerveRequest.SwerveDriveBrake();

    /* Instantiate point (point swerve wheels in a specific direction) */
    point = new SwerveRequest.PointWheelsAt();

    /* Instantiate the Field and Robot Speeds Swerve Requests */
    applyFieldSpeeds =
        new SwerveRequest.ApplyFieldSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withDriveRequestType(DriveRequestType.Velocity);
    applyRobotSpeeds =
        new SwerveRequest.ApplyRobotSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withDriveRequestType(DriveRequestType.Velocity);

    /* Instantiate current heading as empty */
    currentHeading = Optional.empty(); // Keeps track of current heading

    /* Instantiate the rotation last triggered as 0 */
    rotationLastTriggered = 0.0;

    /* Instantiate the logger for telemetry */
    logger = new Telemetry();

    /* Register the telemetry for the swerve drive */
    swerve.registerTelemetry(logger::telemeterize);
  }

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

  public Command swerveBrakeCmd() {
    return swerve.applyRequest(() -> brake);
  }

  public Command seedFieldCentricCmd() {
    return swerve.runOnce(swerve::seedFieldCentric);
  }

  /* Intake Commands */

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

  /* Hopper Commands */

  public Command indexToIntakeCommand() {
    return new InstantCommand(() -> hopper.setDesiredState(HopperState.INDEXTOINTAKE), hopper);
  }

  public Command indexToShooterCommand() {
    return new InstantCommand(() -> hopper.setDesiredState(HopperState.INDEXTOSHOOTER), hopper);
  }

  public Command stopHopperCommand() {
    return new InstantCommand(() -> hopper.setDesiredState(HopperState.STOP), hopper);
  }

  /* Shooter Commands */

  public Command stopShooterCommand() {
    return new InstantCommand(() -> shooter.setDesiredState(ShooterState.STOP), shooter);
  }

  public Command shootCommand() {
    return new InstantCommand(() -> shooter.setDesiredState(ShooterState.SHOOT), shooter);
  }

  public Command prepFuelCommand() {
    return new InstantCommand(() -> shooter.setDesiredState(ShooterState.PREPFUEL), shooter);
  }

  public Command driveSuperstateCommand() {
    return new InstantCommand(() -> setDesiredSuperState(SuperState.DRIVE));
  }

  /* State handling */
  public void setDesiredSuperState(SuperState state) {
    this.state = state;
  }

  public void handleStateTransition() {

    // State machine logic
    switch (state) {
      case DRIVE:
        // Intake remains deployed
        intake.setDesiredState(IntakeState.DEPLOYED);
        // Set Hopper to STOP
        hopper.setDesiredState(HopperState.STOP);
        // Set Shooter to PREPFUEL
        shooter.setDesiredState(ShooterState.STOP);
        break;
      case INTAKE:
        // Set Intake to INTAKE
        intake.setDesiredState(IntakeState.INTAKE);
        // Set Hopper to INDEXTOSHOOTER
        // hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
        // Set Shooter to PREPFUEL
        shooter.setDesiredState(ShooterState.STOP);
        break;
      case OUTTAKE:
        // Set Drive to maintain heading at reduced speeds
        // Set Intake to OUTTAKE
        intake.setDesiredState(IntakeState.OUTTAKE);
        // Set Hopper to INDEXTOINTAKE
        hopper.setDesiredState(HopperState.INDEXTOINTAKE);
        // Set Shooter to PREPFUEL
        shooter.setDesiredState(ShooterState.STOP);
        break;
      case SHOOT:
        // Set Shooter to SHOOT
        shooter.setDesiredState(ShooterState.SHOOT);

        if (shooter.getCurrentState() == ShooterState.SHOOT) {
          // Set Drive to point towards target
          // Once the robot is in position, set to x-lock
          // TODO: In the future, set to hold position, or shoot on the move //
          // Set Intake to AUTO_WOKTOSSING
          intake.setDesiredState(IntakeState.INTAKE);
          // Set Hopper to INDEXTOSHOOTER
          // DogLog.log("Superstructure/Hopper State", hopper.getCurrentState().toString());
          hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
        }
        break;
      case SHOOT_JUICER:
        // Set Shooter to SHOOT
        shooter.setDesiredState(ShooterState.SHOOT);

        if (shooter.getCurrentState() == ShooterState.SHOOT) {
          // Set Drive to point towards target
          // Once the robot is in position, set to x-lock
          // TODO: In the future, set to hold position, or shoot on the move //
          // Set Intake to AUTO_WOKTOSSING
          intake.setDesiredState(IntakeState.JUICER);
          // Set Hopper to INDEXTOSHOOTER
          // DogLog.log("Superstructure/Hopper State", hopper.getCurrentState().toString());
          hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
        }
    }
  }

  @Override
  public void periodic() {
    DogLog.time("Perf/Superstructure");

    // Updates where the robot is //
    Pose2d currentPose = swerve.getState().Pose;

    // Check the distance of the robot pose to the center of the blue hub
    double distanceToBlueHub =
        currentPose.getTranslation().getDistance(FieldConstants.Hub.BLUE_HUB_CENTER_POSE);

    DogLog.log("Superstructure/Distance to Blue Hub (feet)", Units.metersToFeet(distanceToBlueHub));

    DogLog.log("Superstructure/SuperState", state.toString());

    handleStateTransition();

    // Based on where the robot is, update the shooting location the robot should point towards

    DogLog.timeEnd("Perf/Superstructure");
  }
}
