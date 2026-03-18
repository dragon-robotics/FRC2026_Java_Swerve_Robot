// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.util.constants.GeneralConstants.*;
import static frc.robot.util.constants.HopperConstants.HOPPER_ROLLER_DUTY_CYCLE;
import static frc.robot.util.constants.HopperConstants.HOPPER_ROLLER_FOLLOW_MOTOR_ID;
import static frc.robot.util.constants.HopperConstants.HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG;
import static frc.robot.util.constants.HopperConstants.HOPPER_ROLLER_LEAD_MOTOR_ID;
import static frc.robot.util.constants.HopperConstants.HOPPER_ROLLER_LEAD_TALONFX_CONFIG;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_CANCODER_CONFIG;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_DEPLOYED_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_MOTOR_ID;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_STOWED_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_TALONFX_CONFIG;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_DUTY_CYCLE;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_MOTOR_ID;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_TALONFX_CONFIG;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_FOLLOW_MOTOR_ID;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_FOLLOW_TALONFX_CONFIG;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_HOOD_MOTOR_ID;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_HOOD_TALONFX_CONFIG;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_KICKER_DUTY_CYCLE;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_KICKER_MOTOR_ID;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_KICKER_TALONFX_CONFIG;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_LEAD_DUTY_CYCLE;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_LEAD_MOTOR_ID;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_LEAD_TALONFX_CONFIG;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.io.TalonFXMotorIO;
import frc.robot.io.TalonFXMotorIOSim;
import frc.robot.io.TalonFXMotorIOTunable;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystemSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.constants.OperatorConstants;
import frc.robot.util.constants.SwerveConstants;
import frc.robot.util.constants.VisionConstants;

public class RobotContainer {

  /* Robot Subsystems */
  public final CommandSwerveDrivetrain swerveSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  public final HopperSubsystem hopperSubsystem;
  public final ShooterSubsystem shooterSubsystem;
  // public final ClimberSubsystem climberSubsystem;
  //   public final VisionSubsystem visionSubsystem;
  public final Superstructure superstructureSubsystem;

  /* Driver Controllers */
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  /* Swerve Commands */
  private Command defaultDriveCommand;
  private Command shootDriveCommand;
  private Command swerveBrakeCommand;
  private Command seedFieldCentricCommand;

  /* Intake Commands */
  private Command intakeCommand;
  private Command outtakeCommand;
  private Command stowIntakeCommand;
  private Command deployIntakeCommand;

  /* Hopper Commands */
  private Command stopHopperCommand;
  private Command indexToShooterCommand;
  private Command indexToIntakeCommand;

  /* Shooter Commands */
  private Command stopShooterCommand;
  private Command shootCommand;

  /* Climber Commands */
  //   private Command deployClimberCommand;
  //   private Command climbCommand;

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

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
        intakeSubsystem =
            new IntakeSubsystem(
                new TalonFXMotorIO(
                    INTAKE_ROLLER_MOTOR_ID, INTAKE_ROLLER_TALONFX_CONFIG, "Intake Roller"),
                new TalonFXMotorIO(INTAKE_ARM_MOTOR_ID, INTAKE_ARM_TALONFX_CONFIG, "Intake Arm"));
        hopperSubsystem =
            new HopperSubsystem(
                new TalonFXMotorIO(
                    HOPPER_ROLLER_LEAD_MOTOR_ID,
                    HOPPER_ROLLER_LEAD_TALONFX_CONFIG,
                    "Hopper Lead Motor"),
                new TalonFXMotorIO(
                    HOPPER_ROLLER_FOLLOW_MOTOR_ID,
                    HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG,
                    "Hopper Follow Motor",
                    new Follower(HOPPER_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Aligned)));
        shooterSubsystem =
            new ShooterSubsystem(
                new TalonFXMotorIO(
                    SHOOTER_LEAD_MOTOR_ID, SHOOTER_LEAD_TALONFX_CONFIG, "Shooter Lead"),
                new TalonFXMotorIO(
                    SHOOTER_FOLLOW_MOTOR_ID,
                    SHOOTER_FOLLOW_TALONFX_CONFIG,
                    "Shooter Follow",
                    new Follower(SHOOTER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)),
                new TalonFXMotorIO(
                    SHOOTER_KICKER_MOTOR_ID, SHOOTER_KICKER_TALONFX_CONFIG, "Shooter Kicker"),
                new TalonFXMotorIO(
                    SHOOTER_HOOD_MOTOR_ID, SHOOTER_HOOD_TALONFX_CONFIG, "Shooter Hood"));
        // climberSubsystem = new ClimberSubsystem();
        // visionSubsystem =
        //     new VisionSubsystem(
        //         swerveSubsystem,
        //         swerveSubsystem::addVisionMeasurement,
        //         // new VisionIOPhotonVision(
        //         // APTAG_CAMERA_NAMES[0],
        //         // VisionConstants.APTAG_ALIGN_LEFT_CAM_POS,
        //         // swerveSubsystem::getState),
        //         new VisionIOPhotonVision(
        //             APTAG_CAMERA_NAMES[1],
        //             VisionConstants.APTAG_ALIGN_RIGHT_CAM_POS,
        //             swerveSubsystem::getState));
        break;
      case SIM:
        intakeSubsystem =
            new IntakeSubsystemSim(
                new TalonFXMotorIOSim(
                    INTAKE_ROLLER_MOTOR_ID,
                    INTAKE_ROLLER_TALONFX_CONFIG,
                    "KrakenX60_FOC",
                    "Intake Roller"),
                new TalonFXMotorIOSim(
                    INTAKE_ARM_MOTOR_ID,
                    INTAKE_ARM_TALONFX_CONFIG,
                    "KrakenX60_FOC",
                    "Intake Arm",
                    INTAKE_ARM_CANCODER_CONFIG));
        hopperSubsystem =
            new HopperSubsystem(
                new TalonFXMotorIOSim(
                    HOPPER_ROLLER_LEAD_MOTOR_ID,
                    HOPPER_ROLLER_LEAD_TALONFX_CONFIG,
                    "KrakenX60_FOC",
                    "Hopper Lead Motor"),
                new TalonFXMotorIOSim(
                    HOPPER_ROLLER_FOLLOW_MOTOR_ID,
                    HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG,
                    "KrakenX60_FOC",
                    "Shooter Follow",
                    new Follower(HOPPER_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)));
        shooterSubsystem =
            new ShooterSubsystem(
                new TalonFXMotorIOSim(
                    SHOOTER_LEAD_MOTOR_ID,
                    SHOOTER_LEAD_TALONFX_CONFIG,
                    "KrakenX60_FOC",
                    "Shooter Lead"),
                new TalonFXMotorIOSim(
                    SHOOTER_FOLLOW_MOTOR_ID,
                    SHOOTER_FOLLOW_TALONFX_CONFIG,
                    "KrakenX60_FOC",
                    "Shooter Follow",
                    new Follower(SHOOTER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)),
                new TalonFXMotorIOSim(
                    SHOOTER_KICKER_MOTOR_ID,
                    SHOOTER_KICKER_TALONFX_CONFIG,
                    "KrakenX60_FOC",
                    "Shooter Kicker"),
                new TalonFXMotorIOSim(
                    SHOOTER_HOOD_MOTOR_ID,
                    SHOOTER_HOOD_TALONFX_CONFIG,
                    "KrakenX44",
                    "Shooter Hood"));
        // climberSubsystem = new ClimberSubsystem();
        // visionSubsystem =
        //     new VisionSubsystem(
        //         swerveSubsystem,
        //         swerveSubsystem::addVisionMeasurement,
        //         // Auto-Align Cameras //
        //         new VisionIOPhotonVisionSim(
        //             APTAG_CAMERA_NAMES[0],
        //             VisionConstants.APTAG_ALIGN_LEFT_CAM_POS,
        //             swerveSubsystem::getState),
        //         new VisionIOPhotonVisionSim(
        //             APTAG_CAMERA_NAMES[1],
        //             VisionConstants.APTAG_ALIGN_RIGHT_CAM_POS,
        //             swerveSubsystem::getState),
        //         // Apriltag Pose-Estimation Cameras //
        //         new VisionIOPhotonVisionSim(
        //             APTAG_CAMERA_NAMES[2],
        //             VisionConstants.APTAG_POSE_EST_CAM_FL_POS,
        //             swerveSubsystem::getState),
        //         new VisionIOPhotonVisionSim(
        //             APTAG_CAMERA_NAMES[3],
        //             VisionConstants.APTAG_POSE_EST_CAM_FR_POS,
        //             swerveSubsystem::getState),
        //         new VisionIOPhotonVisionSim(
        //             APTAG_CAMERA_NAMES[4],
        //             VisionConstants.APTAG_POSE_EST_CAM_BL_POS,
        //             swerveSubsystem::getState),
        //         new VisionIOPhotonVisionSim(
        //             APTAG_CAMERA_NAMES[5],
        //             VisionConstants.APTAG_POSE_EST_CAM_BR_POS,
        //             swerveSubsystem::getState));
        break;
      case TEST:
        intakeSubsystem =
            new IntakeSubsystem(
                new TalonFXMotorIOTunable(
                    INTAKE_ROLLER_MOTOR_ID, INTAKE_ROLLER_TALONFX_CONFIG, "Intake Roller"),
                new TalonFXMotorIOTunable(
                    INTAKE_ARM_MOTOR_ID, INTAKE_ARM_TALONFX_CONFIG, "Intake Arm"));
        hopperSubsystem =
            new HopperSubsystem(
                new TalonFXMotorIOTunable(
                    HOPPER_ROLLER_LEAD_MOTOR_ID,
                    HOPPER_ROLLER_LEAD_TALONFX_CONFIG,
                    "Hopper Lead Motor"),
                new TalonFXMotorIOTunable(
                    HOPPER_ROLLER_FOLLOW_MOTOR_ID,
                    HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG,
                    "Hopper Follow Motor",
                    new Follower(HOPPER_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Aligned)));
        shooterSubsystem =
            new ShooterSubsystem(
                new TalonFXMotorIOTunable(
                    SHOOTER_LEAD_MOTOR_ID, SHOOTER_LEAD_TALONFX_CONFIG, "Shooter Lead"),
                new TalonFXMotorIOTunable(
                    SHOOTER_FOLLOW_MOTOR_ID,
                    SHOOTER_FOLLOW_TALONFX_CONFIG,
                    "Shooter Follow",
                    new Follower(SHOOTER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)),
                new TalonFXMotorIOTunable(
                    SHOOTER_KICKER_MOTOR_ID, SHOOTER_KICKER_TALONFX_CONFIG, "Shooter Kicker"),
                new TalonFXMotorIOTunable(
                    SHOOTER_HOOD_MOTOR_ID, SHOOTER_HOOD_TALONFX_CONFIG, "Shooter Hood"));
        // climberSubsystem = new ClimberSubsystem();
        // visionSubsystem =
        //     new VisionSubsystem(
        //         swerveSubsystem,
        //         swerveSubsystem::addVisionMeasurement,
        //         // new VisionIOPhotonVision(
        //         // APTAG_CAMERA_NAMES[0],
        //         // VisionConstants.APTAG_ALIGN_LEFT_CAM_POS,
        //         // swerveSubsystem::getState),
        //         new VisionIOPhotonVision(
        //             APTAG_CAMERA_NAMES[1],
        //             VisionConstants.APTAG_ALIGN_RIGHT_CAM_POS,
        //             swerveSubsystem::getState));
        break;
      default: // Default should be in comp mode //
        intakeSubsystem =
            new IntakeSubsystem(
                new TalonFXMotorIO(
                    INTAKE_ROLLER_MOTOR_ID, INTAKE_ROLLER_TALONFX_CONFIG, "Intake Roller"),
                new TalonFXMotorIO(INTAKE_ARM_MOTOR_ID, INTAKE_ARM_TALONFX_CONFIG, "Intake Arm"));
        hopperSubsystem =
            new HopperSubsystem(
                new TalonFXMotorIO(
                    HOPPER_ROLLER_LEAD_MOTOR_ID,
                    HOPPER_ROLLER_LEAD_TALONFX_CONFIG,
                    "Hopper Lead Motor"),
                new TalonFXMotorIO(
                    HOPPER_ROLLER_FOLLOW_MOTOR_ID,
                    HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG,
                    "Hopper Follow Motor",
                    new Follower(HOPPER_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Aligned)));
        shooterSubsystem =
            new ShooterSubsystem(
                new TalonFXMotorIO(
                    SHOOTER_LEAD_MOTOR_ID, SHOOTER_LEAD_TALONFX_CONFIG, "Shooter Lead"),
                new TalonFXMotorIO(
                    SHOOTER_FOLLOW_MOTOR_ID,
                    SHOOTER_FOLLOW_TALONFX_CONFIG,
                    "Shooter Follow",
                    new Follower(SHOOTER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)),
                new TalonFXMotorIO(
                    SHOOTER_KICKER_MOTOR_ID, SHOOTER_KICKER_TALONFX_CONFIG, "Shooter Kicker"),
                new TalonFXMotorIO(
                    SHOOTER_HOOD_MOTOR_ID, SHOOTER_HOOD_TALONFX_CONFIG, "Shooter Hood"));
        // climberSubsystem = new ClimberSubsystem();
        // visionSubsystem =
        //     new VisionSubsystem(
        //         swerveSubsystem,
        //         swerveSubsystem::addVisionMeasurement,
        //         new VisionIOPhotonVision(
        //             APTAG_CAMERA_NAMES[0],
        //             VisionConstants.APTAG_ALIGN_LEFT_CAM_POS,
        //             swerveSubsystem::getState),
        //         new VisionIOPhotonVision(
        //             APTAG_CAMERA_NAMES[1],
        //             VisionConstants.APTAG_ALIGN_RIGHT_CAM_POS,
        //             swerveSubsystem::getState));
        break;
    }

    // Create the superstructure subsystem //
    superstructureSubsystem =
        new Superstructure(
            swerveSubsystem,
            intakeSubsystem,
            hopperSubsystem,
            shooterSubsystem,
            // climberSubsystem,
            null,
            null,
            this);

    defaultDriveCommand =
        superstructureSubsystem.defaultDrive(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> driverController.getHID().getPOV() == 0);

    shootDriveCommand =
        superstructureSubsystem.shootDrive(
            () -> -driverController.getLeftY(), () -> -driverController.getLeftX());

    swerveBrakeCommand = superstructureSubsystem.swerveBrakeCmd();
    seedFieldCentricCommand = superstructureSubsystem.seedFieldCentricCmd();

    // Intake
    intakeCommand = superstructureSubsystem.intakeCommand();
    outtakeCommand = superstructureSubsystem.outtakeCommand();
    deployIntakeCommand = superstructureSubsystem.deployIntakeCommand();
    stowIntakeCommand = superstructureSubsystem.stowIntakeCommand();

    // Hopper
    stopHopperCommand = superstructureSubsystem.stopHopperCommand();
    indexToShooterCommand = superstructureSubsystem.indexToShooterCommand();

    // Shooter
    stopShooterCommand = superstructureSubsystem.stopShooterCommand();
    shootCommand = superstructureSubsystem.shootCommand();

    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  private void configureBindings() {

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(swerveSubsystem.applyRequest(() -> idle).ignoringDisable(true));

    /* Default Commands */
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    swerveSubsystem.setDefaultCommand(defaultDriveCommand);

    /* Driver Controls */

    // Brake the drivetrain on pressing the back button.
    driverController.back().whileTrue(swerveBrakeCommand);

    // Reset the field-centric heading on start button press.
    driverController.start().onTrue(seedFieldCentricCommand);

    /* Intake */
    driverController
        .leftBumper()
        .whileTrue(intakeCommand)
        .whileTrue(indexToShooterCommand)
        .onFalse(deployIntakeCommand)
        .onFalse(stopHopperCommand);

    /* Outtake */
    driverController
        .rightBumper()
        .whileTrue(outtakeCommand)
        .whileTrue(indexToIntakeCommand)
        .onFalse(stopHopperCommand);

    /* Shoot */
    driverController
        .rightTrigger(0.2)
        .whileTrue(shootCommand)
        // .whileTrue(shootDriveCommand)
        .onFalse(stopShooterCommand)
        .onFalse(stopHopperCommand)
        .onFalse(stowIntakeCommand);

    /* Operator Controls */

    /* Manual Intake Arm */
    operatorController
        .a()
        .whileTrue(
            new RunCommand(
                () -> intakeSubsystem.setIntakeArmSetpoint(INTAKE_ARM_DEPLOYED_POSITION, 0),
                intakeSubsystem));

    operatorController
        .b()
        .whileTrue(
            new RunCommand(
                () -> intakeSubsystem.setIntakeArmSetpoint(INTAKE_ARM_STOWED_POSITION, 1),
                intakeSubsystem));

    /* Manual Intake Roller */
    operatorController
        .leftBumper()
        .whileTrue(
            new RunCommand(
                () -> intakeSubsystem.runIntakeRollerPercentage(INTAKE_ROLLER_DUTY_CYCLE),
                intakeSubsystem))
        .whileFalse(
            new RunCommand(() -> intakeSubsystem.runIntakeRollerPercentage(0), intakeSubsystem));

    operatorController
        .rightBumper()
        .whileTrue(
            new RunCommand(
                () -> intakeSubsystem.runIntakeRollerPercentage(-INTAKE_ROLLER_DUTY_CYCLE),
                intakeSubsystem))
        .whileFalse(
            new RunCommand(() -> intakeSubsystem.runIntakeRollerPercentage(0), intakeSubsystem));

    /* Manual Hopper Roller */
    operatorController
        .leftStick()
        .whileTrue(
            new RunCommand(
                () -> hopperSubsystem.runHopperRollerPercentage(HOPPER_ROLLER_DUTY_CYCLE),
                hopperSubsystem))
        .whileFalse(
            new RunCommand(() -> hopperSubsystem.runHopperRollerPercentage(0), hopperSubsystem));

    operatorController
        .rightStick()
        .whileTrue(
            new RunCommand(
                () -> hopperSubsystem.runHopperRollerPercentage(-HOPPER_ROLLER_DUTY_CYCLE),
                hopperSubsystem))
        .whileFalse(
            new RunCommand(() -> hopperSubsystem.runHopperRollerPercentage(0), hopperSubsystem));

    /* Manual Shooter Kicker */
    operatorController
        .leftTrigger(0.2)
        .whileTrue(
            new RunCommand(
                () -> shooterSubsystem.runKickerMotorPercentage(SHOOTER_KICKER_DUTY_CYCLE),
                shooterSubsystem))
        .whileFalse(
            new RunCommand(() -> shooterSubsystem.runKickerMotorPercentage(0), shooterSubsystem));

    operatorController
        .rightTrigger(0.2)
        .whileTrue(
            new RunCommand(
                () -> shooterSubsystem.runKickerMotorPercentage(-SHOOTER_KICKER_DUTY_CYCLE),
                shooterSubsystem))
        .whileFalse(
            new RunCommand(() -> shooterSubsystem.runKickerMotorPercentage(0), shooterSubsystem));

    /* Manual Shooter Flywheel */
    operatorController
        .x()
        .whileTrue(
            new RunCommand(
                () -> shooterSubsystem.runShooterMotorPercentage(SHOOTER_LEAD_DUTY_CYCLE),
                shooterSubsystem))
        .whileFalse(
            new RunCommand(() -> shooterSubsystem.runShooterMotorPercentage(0), shooterSubsystem));

    operatorController
        .y()
        .whileTrue(
            new RunCommand(
                () -> shooterSubsystem.runShooterMotorPercentage(SHOOTER_LEAD_DUTY_CYCLE),
                shooterSubsystem))
        .whileFalse(
            new RunCommand(() -> shooterSubsystem.runShooterMotorPercentage(0), shooterSubsystem));

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    // driverController
    //     .back()
    //     .and(driverController.y())
    //     .whileTrue(swerveSubsystem.sysIdDynamic(Direction.kForward));
    // driverController
    //     .back()
    //     .and(driverController.x())
    //     .whileTrue(swerveSubsystem.sysIdDynamic(Direction.kReverse));
    // driverController
    //     .start()
    //     .and(driverController.y())
    //     .whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kForward));
    // driverController
    //     .start()
    //     .and(driverController.x())
    //     .whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kReverse));

    // driverController.b().onTrue(Commands.runOnce(() -> SignalLogger.stop()));
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }
}
