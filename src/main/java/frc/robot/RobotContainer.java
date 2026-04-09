// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.util.constants.GeneralConstants.*;
import static frc.robot.util.constants.HopperConstants.HOPPER_ROLLER_FOLLOW_MOTOR_ID;
import static frc.robot.util.constants.HopperConstants.HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG;
import static frc.robot.util.constants.HopperConstants.HOPPER_ROLLER_LEAD_MOTOR_ID;
import static frc.robot.util.constants.HopperConstants.HOPPER_ROLLER_LEAD_TALONFX_CONFIG;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_CANCODER_CONFIG;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_MOTOR_ID;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_TALONFX_CONFIG;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_MOTOR_ID;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_TALONFX_CONFIG;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_FOLLOW_MOTOR_ID;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_FOLLOW_TALONFX_CONFIG;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_HOOD_MOTOR_ID;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_HOOD_TALONFX_CONFIG;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_KICKER_MOTOR_ID;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_KICKER_TALONFX_CONFIG;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_LEAD_MOTOR_ID;
import static frc.robot.util.constants.ShooterConstants.SHOOTER_LEAD_TALONFX_CONFIG;
import static frc.robot.util.constants.VisionConstants.APTAG_CAMERA_NAMES;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.io.TalonFXMotorIO;
import frc.robot.io.TalonFXMotorIOSim;
import frc.robot.io.TalonFXMotorIOTunable;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.intake.IntakeSubsystemSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;
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
  public final VisionSubsystem visionSubsystem;
  public final Superstructure superstructureSubsystem;

  /* Driver Controllers */
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  /* Swerve Commands */
  private Command defaultDriveCommand;
  private Command shootDriveCommand;
  private Command swerveBrakeCommand;
  private Command seedFieldCentricCommand;
  private Command driveCommand;

  /* Intake Commands */
  private Command intakeCommand;

  /* Shooter Commands */
  private Command shootCommand;

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    /* Setup DogLog */
    DogLog.setOptions(new DogLogOptions());
    // DogLog.setPdh(new PowerDistribution());

    /* Disable warnings */
    DriverStation.silenceJoystickConnectionWarning(true);

    /* Initialize Joysticks */
    driverController = new CommandXboxController(OperatorConstants.DRIVER_PORT);
    operatorController = new CommandXboxController(OperatorConstants.OPERATOR_PORT);

    /* Initialize Subsystems */
    swerveSubsystem = TunerConstants.createDrivetrain(
        250, SwerveConstants.ODOMETRY_STD, VisionConstants.DEFAULT_TAG_STDDEV);

    // Change initialization based on the state of the robot //
    switch (CURRENT_MODE) {
      case COMP:
        intakeSubsystem = new IntakeSubsystem(
            new TalonFXMotorIO(
                INTAKE_ROLLER_MOTOR_ID, INTAKE_ROLLER_TALONFX_CONFIG, "Intake Roller"),
            new TalonFXMotorIO(INTAKE_ARM_MOTOR_ID, INTAKE_ARM_TALONFX_CONFIG, "Intake Arm"));
        hopperSubsystem = new HopperSubsystem(
            new TalonFXMotorIO(
                HOPPER_ROLLER_LEAD_MOTOR_ID,
                HOPPER_ROLLER_LEAD_TALONFX_CONFIG,
                "Hopper Lead Motor"),
            new TalonFXMotorIO(
                HOPPER_ROLLER_FOLLOW_MOTOR_ID,
                HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG,
                "Hopper Follow Motor",
                new Follower(HOPPER_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Aligned)));
        shooterSubsystem = new ShooterSubsystem(
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
        visionSubsystem = new VisionSubsystem(
            swerveSubsystem,
            swerveSubsystem::addVisionMeasurement,
            new VisionIOPhotonVision(
                APTAG_CAMERA_NAMES[0], VisionConstants.APTAG_POSE_EST_CAM_F_POS),
            new VisionIOPhotonVision(
                APTAG_CAMERA_NAMES[1], VisionConstants.APTAG_POSE_EST_CAM_R_POS),
            new VisionIOPhotonVision(
                APTAG_CAMERA_NAMES[2], VisionConstants.APTAG_POSE_EST_CAM_B_POS),
            new VisionIOPhotonVision(
                APTAG_CAMERA_NAMES[3], VisionConstants.APTAG_POSE_EST_CAM_L_POS));
        break;
      case SIM:
        intakeSubsystem = new IntakeSubsystemSim(
            new TalonFXMotorIOSim(
                INTAKE_ROLLER_MOTOR_ID,
                INTAKE_ROLLER_TALONFX_CONFIG,
                "KrakenX60",
                "Intake Roller"),
            new TalonFXMotorIOSim(
                INTAKE_ARM_MOTOR_ID,
                INTAKE_ARM_TALONFX_CONFIG,
                "KrakenX60",
                "Intake Arm",
                INTAKE_ARM_CANCODER_CONFIG));
        hopperSubsystem = new HopperSubsystem(
            new TalonFXMotorIOSim(
                HOPPER_ROLLER_LEAD_MOTOR_ID,
                HOPPER_ROLLER_LEAD_TALONFX_CONFIG,
                "KrakenX60",
                "Hopper Lead Motor"),
            new TalonFXMotorIOSim(
                HOPPER_ROLLER_FOLLOW_MOTOR_ID,
                HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG,
                "KrakenX60",
                "Hopper Follow Motor",
                new Follower(HOPPER_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)));
        shooterSubsystem = new ShooterSubsystem(
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
        visionSubsystem = new VisionSubsystem(
            swerveSubsystem,
            swerveSubsystem::addVisionMeasurement,
            // // Apriltag Pose-Estimation Cameras //
            new VisionIOPhotonVisionSim(
                APTAG_CAMERA_NAMES[0],
                VisionConstants.APTAG_POSE_EST_CAM_F_POS,
                () -> swerveSubsystem.getState().Pose),
            new VisionIOPhotonVisionSim(
                APTAG_CAMERA_NAMES[1],
                VisionConstants.APTAG_POSE_EST_CAM_R_POS,
                () -> swerveSubsystem.getState().Pose),
            new VisionIOPhotonVisionSim(
                APTAG_CAMERA_NAMES[2],
                VisionConstants.APTAG_POSE_EST_CAM_B_POS,
                () -> swerveSubsystem.getState().Pose),
            new VisionIOPhotonVisionSim(
                APTAG_CAMERA_NAMES[3],
                VisionConstants.APTAG_POSE_EST_CAM_L_POS,
                () -> swerveSubsystem.getState().Pose));
        break;
      case TEST:
        intakeSubsystem = new IntakeSubsystem(
            new TalonFXMotorIOTunable(
                INTAKE_ROLLER_MOTOR_ID, INTAKE_ROLLER_TALONFX_CONFIG, "Intake Roller"),
            new TalonFXMotorIOTunable(
                INTAKE_ARM_MOTOR_ID, INTAKE_ARM_TALONFX_CONFIG, "Intake Arm"));
        hopperSubsystem = new HopperSubsystem(
            new TalonFXMotorIOTunable(
                HOPPER_ROLLER_LEAD_MOTOR_ID,
                HOPPER_ROLLER_LEAD_TALONFX_CONFIG,
                "Hopper Lead Motor"),
            new TalonFXMotorIOTunable(
                HOPPER_ROLLER_FOLLOW_MOTOR_ID,
                HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG,
                "Hopper Follow Motor",
                new Follower(HOPPER_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Aligned)));
        shooterSubsystem = new ShooterSubsystem(
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
        visionSubsystem = new VisionSubsystem(
            swerveSubsystem,
            swerveSubsystem::addVisionMeasurement,
            new VisionIOPhotonVision(
                APTAG_CAMERA_NAMES[0], VisionConstants.APTAG_POSE_EST_CAM_F_POS),
            new VisionIOPhotonVision(
                APTAG_CAMERA_NAMES[1], VisionConstants.APTAG_POSE_EST_CAM_R_POS),
            new VisionIOPhotonVision(
                APTAG_CAMERA_NAMES[2], VisionConstants.APTAG_POSE_EST_CAM_B_POS),
            new VisionIOPhotonVision(
                APTAG_CAMERA_NAMES[3], VisionConstants.APTAG_POSE_EST_CAM_L_POS));
        break;
      default: // Default should be in comp mode //
        intakeSubsystem = new IntakeSubsystem(
            new TalonFXMotorIO(
                INTAKE_ROLLER_MOTOR_ID, INTAKE_ROLLER_TALONFX_CONFIG, "Intake Roller"),
            new TalonFXMotorIO(INTAKE_ARM_MOTOR_ID, INTAKE_ARM_TALONFX_CONFIG, "Intake Arm"));
        hopperSubsystem = new HopperSubsystem(
            new TalonFXMotorIO(
                HOPPER_ROLLER_LEAD_MOTOR_ID,
                HOPPER_ROLLER_LEAD_TALONFX_CONFIG,
                "Hopper Lead Motor"),
            new TalonFXMotorIO(
                HOPPER_ROLLER_FOLLOW_MOTOR_ID,
                HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG,
                "Hopper Follow Motor",
                new Follower(HOPPER_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Aligned)));
        shooterSubsystem = new ShooterSubsystem(
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
        visionSubsystem = new VisionSubsystem(
            swerveSubsystem,
            swerveSubsystem::addVisionMeasurement,
            new VisionIOPhotonVision(
                APTAG_CAMERA_NAMES[0], VisionConstants.APTAG_POSE_EST_CAM_F_POS),
            new VisionIOPhotonVision(
                APTAG_CAMERA_NAMES[1], VisionConstants.APTAG_POSE_EST_CAM_R_POS),
            new VisionIOPhotonVision(
                APTAG_CAMERA_NAMES[2], VisionConstants.APTAG_POSE_EST_CAM_B_POS),
            new VisionIOPhotonVision(
                APTAG_CAMERA_NAMES[3], VisionConstants.APTAG_POSE_EST_CAM_L_POS));
        break;
    }

    // Create the superstructure subsystem //
    superstructureSubsystem = new Superstructure(
        swerveSubsystem,
        intakeSubsystem,
        hopperSubsystem,
        shooterSubsystem,
        visionSubsystem,
        this);

    defaultDriveCommand = superstructureSubsystem.defaultDrive(
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> driverController.getHID().getPOV() == 0,
        () -> driverController.getHID().getXButton());

    shootDriveCommand = superstructureSubsystem.shootDrive(
        () -> -driverController.getLeftY(), () -> -driverController.getLeftX());

    swerveBrakeCommand = superstructureSubsystem.swerveBrakeCmd();
    seedFieldCentricCommand = superstructureSubsystem.seedFieldCentricCmd();

    // Intake
    intakeCommand = superstructureSubsystem.setStateCmd(SuperState.INTAKE);
    // Shoot
    shootCommand = superstructureSubsystem
        .setStateCmd(SuperState.SHOOT)
        .alongWith(superstructureSubsystem.aimAtTargetPose())
        .alongWith(
            Commands.waitSeconds(1.5)
                .andThen(superstructureSubsystem.intakeOverrideCmd(IntakeState.JUICER)));
    driveCommand = superstructureSubsystem.setStateCmd(SuperState.DRIVE);

    NamedCommands.registerCommand("Intake", intakeCommand);
    NamedCommands.registerCommand("Shoot", shootCommand);
    NamedCommands.registerCommand("Drive", driveCommand);

    autoChooser = AutoBuilder.buildAutoChooser();
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

    // Reset the field-centric heading on both start and back button press.
    driverController.start().and(driverController.back()).onTrue(seedFieldCentricCommand);

    /* Rotate the robot to the nearest diamond angle for bump traversal */

    /* Intake */
    driverController
        .leftTrigger(0.2)
        .onTrue(superstructureSubsystem.setStateCmd(SuperState.INTAKE))
        .onFalse(superstructureSubsystem.setStateCmd(SuperState.DRIVE));

    /* Outtake */
    driverController
        .rightBumper()
        .onTrue(superstructureSubsystem.setStateCmd(SuperState.OUTTAKE))
        .onFalse(superstructureSubsystem.setStateCmd(SuperState.DRIVE));

    /* Shoot */
    driverController
        .rightTrigger(0.2)
        .onTrue(superstructureSubsystem.setStateCmd(SuperState.SHOOT))
        .whileTrue(superstructureSubsystem.aimAtTargetPose())
        .onFalse(superstructureSubsystem.setStateCmd(SuperState.DRIVE));

    /* Driver can control the Juicer mode if needed */
    driverController
        .b()
        .whileTrue(superstructureSubsystem.intakeOverrideCmd(IntakeState.JUICER))
        .onFalse(superstructureSubsystem.intakeOverrideCmd(IntakeState.DEPLOYED));

    /* Operator Controls */

    /* Reset the robot pose based on vision — operator back and start button */
    operatorController
        .start()
        .and(operatorController.back())
        .onTrue(superstructureSubsystem.forceReseedFromVisionCmd());

    /* Operator mainly controls the Juicer mode */
    operatorController
        .b()
        .whileTrue(superstructureSubsystem.intakeOverrideCmd(IntakeState.JUICER))
        .onFalse(superstructureSubsystem.intakeOverrideCmd(IntakeState.DEPLOYED));

    /* TODO: Add overrides to the shooter in case vision goes down */
    // If up d-pad, shooter is close up bumper shot
    // If left or right d-pad, shooter is trench shot
    // If down d-pad, shooter is tower shot

    // /* Manual Intake Arm */
    // operatorController
    // .a()
    // .whileTrue(
    // new RunCommand(
    // () -> intakeSubsystem.setIntakeArmSetpoint(INTAKE_ARM_DEPLOYED_POSITION, 0),
    // intakeSubsystem));

    // operatorController.b().whileTrue(wokTossIntakeCommand).onFalse(deployIntakeCommand);

    // /* Manual Intake Roller */
    // operatorController
    // .leftBumper()
    // .whileTrue(
    // new RunCommand(
    // () -> intakeSubsystem.runIntakeRollerPercentage(INTAKE_ROLLER_DUTY_CYCLE),
    // intakeSubsystem))
    // .whileFalse(
    // new RunCommand(() -> intakeSubsystem.runIntakeRollerPercentage(0),
    // intakeSubsystem));

    // operatorController
    // .rightBumper()
    // .whileTrue(
    // new RunCommand(
    // () -> intakeSubsystem.runIntakeRollerPercentage(-INTAKE_ROLLER_DUTY_CYCLE),
    // intakeSubsystem))
    // .whileFalse(
    // new RunCommand(() -> intakeSubsystem.runIntakeRollerPercentage(0),
    // intakeSubsystem));

    // /* Manual Hopper Roller */
    // operatorController
    // .leftStick()
    // .whileTrue(
    // new RunCommand(
    // () -> hopperSubsystem.runHopperRollerPercentage(HOPPER_ROLLER_DUTY_CYCLE),
    // hopperSubsystem))
    // .whileFalse(
    // new RunCommand(() -> hopperSubsystem.runHopperRollerPercentage(0),
    // hopperSubsystem));

    // operatorController
    // .rightStick()
    // .whileTrue(
    // new RunCommand(
    // () -> hopperSubsystem.runHopperRollerPercentage(-HOPPER_ROLLER_DUTY_CYCLE),
    // hopperSubsystem))
    // .whileFalse(
    // new RunCommand(() -> hopperSubsystem.runHopperRollerPercentage(0),
    // hopperSubsystem));

    // /* Manual Shooter Kicker */
    // operatorController
    // .leftTrigger(0.2)
    // .whileTrue(
    // new RunCommand(
    // () -> shooterSubsystem.runKickerMotorPercentage(SHOOTER_KICKER_DUTY_CYCLE),
    // shooterSubsystem))
    // .whileFalse(
    // new RunCommand(() -> shooterSubsystem.runKickerMotorPercentage(0),
    // shooterSubsystem));

    // operatorController
    // .rightTrigger(0.2)
    // .whileTrue(
    // new RunCommand(
    // () -> shooterSubsystem.runKickerMotorPercentage(-SHOOTER_KICKER_DUTY_CYCLE),
    // shooterSubsystem))
    // .whileFalse(
    // new RunCommand(() -> shooterSubsystem.runKickerMotorPercentage(0),
    // shooterSubsystem));

    // /* Manual Shooter Flywheel */
    // operatorController
    // .x()
    // .whileTrue(
    // new RunCommand(
    // () -> shooterSubsystem.runShooterMotorPercentage(SHOOTER_LEAD_DUTY_CYCLE),
    // shooterSubsystem))
    // .whileFalse(
    // new RunCommand(() -> shooterSubsystem.runShooterMotorPercentage(0),
    // shooterSubsystem));

    // operatorController
    // .y()
    // .whileTrue(
    // new RunCommand(
    // () -> shooterSubsystem.runShooterMotorPercentage(SHOOTER_LEAD_DUTY_CYCLE),
    // shooterSubsystem))
    // .whileFalse(
    // new RunCommand(() -> shooterSubsystem.runShooterMotorPercentage(0),
    // shooterSubsystem));

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    // driverController
    // .back()
    // .and(driverController.y())
    // .whileTrue(swerveSubsystem.sysIdDynamic(Direction.kForward));
    // driverController
    // .back()
    // .and(driverController.x())
    // .whileTrue(swerveSubsystem.sysIdDynamic(Direction.kReverse));
    // driverController
    // .start()
    // .and(driverController.y())
    // .whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kForward));
    // driverController
    // .start()
    // .and(driverController.x())
    // .whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kReverse));

    // driverController.b().onTrue(Commands.runOnce(() -> SignalLogger.stop()));
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }
}
