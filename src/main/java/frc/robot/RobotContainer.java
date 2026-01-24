// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.VisionConstants.APTAG_CAMERA_NAMES;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  /* Robot Subsystems */
  public final CommandSwerveDrivetrain swerveSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  public final HopperSubsystem hopperSubsystem;
  public final ShooterSubsystem shooterSubsystem;
  public final ClimberSubsystem climberSubsystem;
  public final VisionSubsystem visionSubsystem;
  public final Superstructure superstructureSubsystem;  

  /* Driver Controllers */
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  /* Swerve Commands */
  private Command defaultDriveCommand;
  private Command driveAimedAtHubCommand;
  private Command swerveBrakeCommand;
  private Command seedFieldCentralCommand;

  /* Intake Commands */
  private Command intakeCommand;

  /* Shooter Commands */
  private Command shootCommand;

  /* Climber Commands */
  private Command deployClimberCommand;
  private Command climbCommand;

  private double MaxSpeed =
      1.0
          * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
          * 0.5; // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(OperatorConstants.DRIVER_PORT);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {

    /* Setup DogLog */
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    DogLog.setPdh(new PowerDistribution());

    /* Initialize Joysticks */
    driverController = new CommandXboxController(OperatorConstants.DRIVER_PORT);
    operatorController = new CommandXboxController(OperatorConstants.OPERATOR_PORT);

    /* Initialize Subsystems */
    swerveSubsystem =
        TunerConstants.createDrivetrain(
            250, SwerveConstants.ODOMETRY_STD, VisionConstants.DEFAULT_TAG_STDDEV);

    // Change initialization based on the state of the robot //
    switch (CURRENT_MODE) {
      case COMP:
        intakeSubsystem = new IntakeSubsystem();
        hopperSubsystem = new HopperSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        climberSubsystem = new ClimberSubsystem();
        visionSubsystem =
            new VisionSubsystem(
                swerveSubsystem,
                swerveSubsystem::addVisionMeasurement,
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[0],
                    VisionConstants.APTAG_ALIGN_LEFT_CAM_POS,
                    swerveSubsystem::getState),
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[1],
                    VisionConstants.APTAG_ALIGN_RIGHT_CAM_POS,
                    swerveSubsystem::getState));
        break;
      case SIM:
        intakeSubsystem = new IntakeSubsystem();
        hopperSubsystem = new HopperSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        climberSubsystem = new ClimberSubsystem();
        visionSubsystem =
            new VisionSubsystem(
                swerveSubsystem,
                swerveSubsystem::addVisionMeasurement,
                // Auto-Align Cameras //
                new VisionIOPhotonVisionSim(
                    APTAG_CAMERA_NAMES[0],
                    VisionConstants.APTAG_ALIGN_LEFT_CAM_POS,
                    swerveSubsystem::getState),
                new VisionIOPhotonVisionSim(
                    APTAG_CAMERA_NAMES[1],
                    VisionConstants.APTAG_ALIGN_RIGHT_CAM_POS,
                    swerveSubsystem::getState)
                // // Apriltag Pose-Estimation Cameras //
                // new VisionIOPhotonVisionSim(
                //     APTAG_CAMERA_NAMES[2],
                //     VisionConstants.APTAG_POSE_EST_CAFL_POS,
                //     swerveSubsystem::getState),
                // new VisionIOPhotonVisionSim(
                //     APTAG_CAMERA_NAMES[3],
                //     VisionConstants.APTAG_POSE_EST_CAFR_POS,
                //     swerveSubsystem::getState),
                // new VisionIOPhotonVisionSim(
                //     APTAG_CAMERA_NAMES[4],
                //     VisionConstants.APTAG_POSE_EST_CABL_POS,
                //     swerveSubsystem::getState),
                // new VisionIOPhotonVisionSim(
                //     APTAG_CAMERA_NAMES[5],
                //     VisionConstants.APTAG_POSE_EST_CABR_POS,
                //     swerveSubsystem::getState)
                );
        break;
      case TEST:
        intakeSubsystem = new IntakeSubsystem();
        hopperSubsystem = new HopperSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        climberSubsystem = new ClimberSubsystem();
        visionSubsystem =
            new VisionSubsystem(
                swerveSubsystem,
                swerveSubsystem::addVisionMeasurement,
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[0],
                    VisionConstants.APTAG_ALIGN_LEFT_CAM_POS,
                    swerveSubsystem::getState),
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[1],
                    VisionConstants.APTAG_ALIGN_RIGHT_CAM_POS,
                    swerveSubsystem::getState));
        break;
      default: // Default should be in comp mode //
        intakeSubsystem = new IntakeSubsystem();
        hopperSubsystem = new HopperSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        climberSubsystem = new ClimberSubsystem();
        visionSubsystem =
            new VisionSubsystem(
                swerveSubsystem,
                swerveSubsystem::addVisionMeasurement,
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[0],
                    VisionConstants.APTAG_ALIGN_LEFT_CAM_POS,
                    swerveSubsystem::getState),
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[1],
                    VisionConstants.APTAG_ALIGN_RIGHT_CAM_POS,
                    swerveSubsystem::getState));
        break;
    }

    // Create the superstructure subsystem //
    superstructureSubsystem =
        new Superstructure(
            swerveSubsystem,
            intakeSubsystem,
            hopperSubsystem,
            shooterSubsystem,
            climberSubsystem,
            visionSubsystem,
            this);

    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    // Simple drive forward auton
    final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        // Then slowly drive forward (away from us) for 5 seconds.
        drivetrain
            .applyRequest(() -> drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
            .withTimeout(5.0),
        // Finally idle for the rest of auton
        drivetrain.applyRequest(() -> idle));
  }
}
