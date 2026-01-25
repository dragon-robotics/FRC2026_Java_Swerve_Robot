package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Superstructure extends SubsystemBase {

  private final CommandSwerveDrivetrain swerve;
  private final IntakeSubsystem intake;
  private final HopperSubsystem hopper;
  private final ShooterSubsystem shooter;
  private final ClimberSubsystem climber;
  private final VisionSubsystem vision;
  private final RobotContainer container;
  private final Telemetry logger;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive;
  private final SwerveRequest.SwerveDriveBrake brake;
  private final SwerveRequest.PointWheelsAt point;
  private final SwerveRequest.RobotCentric driveRobotCentric;
  private final SwerveRequest.FieldCentricFacingAngle driveMaintainHeading;

  /* Used for path following */
  private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds;
  private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds;

  private final double maxSpeed;
  private final double maxAngularRate;

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

    /* Instatiate swerve max speed and angular rate */
    maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Max speed at 12 volts
    maxAngularRate =
        RotationsPerSecond.of(1).in(RadiansPerSecond); // 1 rotation per second max angular velocity

    /* Instantiate default field centric drive (no need to maintain heading) */
    drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.05)
            .withRotationalDeadband(maxAngularRate * 0.05) // Add a 5% deadband
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withDesaturateWheelSpeeds(true); // Desaturate wheel speeds to prevent clipping

    /* Instantiate brake (X-lock swerve wheels) */
    brake = new SwerveRequest.SwerveDriveBrake();

    /* Instantiate point (point swerve wheels in a specific direction) */
    point = new SwerveRequest.PointWheelsAt();

    /* Instantiate robot centric drive (forward is based on forward pose of the
    robot) */
    driveRobotCentric =
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* Instantiate field centric drive (maintain heading) */
    driveMaintainHeading =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.05)
            .withRotationalDeadband(maxAngularRate * 0.05)
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withDesaturateWheelSpeeds(true);

    /* Set the PID constants for the Maintain Heading controller */
    driveMaintainHeading.HeadingController.setPID(
        SwerveConstants.HEADING_KP, SwerveConstants.HEADING_KI, SwerveConstants.HEADING_KD);
    driveMaintainHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveMaintainHeading.HeadingController.setTolerance(SwerveConstants.HEADING_TOLERANCE);

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
    logger = new Telemetry(maxSpeed);

    /* Register the telemetry for the swerve drive */
    swerve.registerTelemetry(logger::telemeterize);
  }

  public Command defaultDriveCmd(
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier halfSpeedSup) {
    return new RunCommand(
        () -> {
          double rawRotation = rotationSup.getAsDouble();
          var speeds =
              processJoystickInputs(
                  translationSup.getAsDouble(),
                  strafeSup.getAsDouble(),
                  rawRotation,
                  halfSpeedSup.getAsBoolean());

          boolean rotationTriggered = Math.abs(rawRotation) > SwerveConstants.SWERVE_DEADBAND;
          boolean rotationActive =
              MathUtil.isNear(rotationLastTriggered, Timer.getFPGATimestamp(), 0.1)
                  && (Math.abs(swerve.getState().Speeds.omegaRadiansPerSecond)
                      > Math.toRadians(10));

          if (rotationTriggered) {
            rotationLastTriggered = Timer.getFPGATimestamp();
          }

          if (rotationTriggered || rotationActive) {
            setSwerveToRotate(speeds.translation, speeds.strafe, speeds.rotation);
          } else {
            setSwerveToMaintainHeading(speeds.translation, speeds.strafe);
          }
        },
        swerve);
  }

  private static class Speeds {
    double translation;
    double strafe;
    double rotation;
  }

  private Speeds processJoystickInputs(
      double rawTranslation, double rawStrafe, double rawRotation, boolean halfSpeed) {
    double translation = MathUtil.applyDeadband(rawTranslation, SwerveConstants.SWERVE_DEADBAND, 1);
    double strafe = MathUtil.applyDeadband(rawStrafe, SwerveConstants.SWERVE_DEADBAND, 1);
    double rotation = MathUtil.applyDeadband(rawRotation, SwerveConstants.SWERVE_DEADBAND, 1);

    translation = Math.copySign(translation * translation, translation);
    strafe = Math.copySign(strafe * strafe, strafe);
    rotation = Math.copySign(rotation * rotation, rotation);

    if (halfSpeed) {
      translation *= 0.35;
      strafe *= 0.35;
      rotation *= 0.35;
    }

    Speeds speeds = new Speeds();
    speeds.translation = translation * maxSpeed;
    speeds.strafe = strafe * maxSpeed;
    speeds.rotation = rotation * maxAngularRate;
    return speeds;
  }

  private void setSwerveToRotate(double translation, double strafe, double rotation) {
    swerve.setControl(
        drive.withVelocityX(translation).withVelocityY(strafe).withRotationalRate(rotation));
    currentHeading = Optional.empty();
  }

  private void setSwerveToMaintainHeading(double translation, double strafe) {
    if (currentHeading.isEmpty()) {
      currentHeading = Optional.of(swerve.getState().Pose.getRotation());
    }

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      Rotation2d targetDirection =
          alliance.get() == Alliance.Blue
              ? currentHeading.get()
              : currentHeading.get().rotateBy(Rotation2d.fromDegrees(180));
      swerve.setControl(
          driveMaintainHeading
              .withVelocityX(translation)
              .withVelocityY(strafe)
              .withTargetDirection(targetDirection));
    }
  }

  public Command swerveBrakeCmd() {
    return swerve.applyRequest(() -> brake);
  }

  public Command seedFieldCentricCmd() {
    return swerve.runOnce(swerve::seedFieldCentric);
  }
}
