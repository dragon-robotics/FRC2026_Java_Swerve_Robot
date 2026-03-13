package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.DefaultDriveCmd;
import frc.robot.commands.ShootDriveCmd;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem.HopperState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.Telemetry;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Superstructure extends SubsystemBase {

  public enum SuperState {
    DRIVE,            // DRIVE
    INTAKE,           // DRIVE, SHOOTER PREPFUEL, HOPPER INDEXTOSHOOTER, INTAKE INTAKE
    OUTTAKE,          // DRIVE, SHOOTER PREPFUEL, HOPPER INDEXTOINTAKE, INTAKE OUTTAKE
    SHOOT,            // DRIVE points towards target, SHOOTER SHOOT, HOPPER INDEXTOSHOOTER, INTAKE OFF
  }

  private SuperState state;

  private final CommandSwerveDrivetrain swerve;
  private final IntakeSubsystem intake;
  private final HopperSubsystem hopper;
  private final ShooterSubsystem shooter;
  private final ClimberSubsystem climber;
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
      ClimberSubsystem climber,
      VisionSubsystem vision,
      RobotContainer container) {
    this.swerve = swerve;
    this.intake = intake;
    this.hopper = hopper;
    this.shooter = shooter;
    this.climber = climber;
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

  /* State handling */
  public void setDesiredSuperState(SuperState state) {
    this.state = state;
  }

  public void handleStateTransition() {
    // State machine logic
    switch (state) {
      case DRIVE:
        // Set Drive to maintain heading

        // Intake remains in deployed or home based on current intake state
        // Set Hopper to STOP
        // Set Shooter to PREPFUEL
        break;
      case INTAKE:
        // Set Drive to maintain heading at reduced speeds
        // Set Intake to INTAKE
        // Set Hopper to INDEXTOSHOOTER
        // Set Shooter to PREPFUEL
        break;
      case OUTTAKE:
        // Set Drive to maintain heading at reduced speeds
        // Set Intake to OUTTAKE
        // Set Hopper to INDEXTOINTAKE
        // Set Shooter to PREPFUEL
        break;
      case SHOOT:
        // Set Drive to point towards target
        // Set Intake to INTAKE
        // Set Hopper to INDEXTOSHOOTER
        // Set Shooter to SHOOT
        break;
    }
  }

  @Override
  public void periodic() {}
}
