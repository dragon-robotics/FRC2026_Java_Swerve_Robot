// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.util.constants.GeneralConstants.*;
import static frc.robot.util.constants.HopperConstants.HOPPER_ROLLER_MOTOR_ID;
import static frc.robot.util.constants.HopperConstants.HOPPER_ROLLER_TALONFX_CONFIG;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_CANCODER_CONFIG;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_DEPLOYED_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_STOWED_POSITION;
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

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.hopper.HopperIOTalonFXSim;
import frc.robot.subsystems.hopper.HopperIOTalonFXTunable;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeIOTalonFXSim;
import frc.robot.subsystems.intake.IntakeIOTalonFXTunable;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystemSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterIOTalonFXSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFXTunable;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.constants.OperatorConstants;
import frc.robot.util.constants.SwerveConstants;
import frc.robot.util.constants.VisionConstants;
import java.util.Optional;

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

  /* Manual override commands */
  
  private Command unjamCommand;
  private Command reverseShooterCommand;
  private Command reverseHopperCommand;

  private double intakeRollerSpeed = -1.0;
  private double hopperRollerSpeed = -0.5;
  private double shooterLeadSpeed = -0.5;
  private double shooterKickerSpeed = -0.5;

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
                new IntakeIOTalonFX(
                    INTAKE_ROLLER_MOTOR_ID, INTAKE_ROLLER_TALONFX_CONFIG, "Intake Roller"),
                new IntakeIOTalonFX(INTAKE_ARM_MOTOR_ID, INTAKE_ARM_TALONFX_CONFIG, "Intake Arm"));
        hopperSubsystem =
            new HopperSubsystem(
                new HopperIOTalonFX(
                    HOPPER_ROLLER_MOTOR_ID, HOPPER_ROLLER_TALONFX_CONFIG, "Hopper Motor"));
        shooterSubsystem =
            new ShooterSubsystem(
                new ShooterIOTalonFX(
                    SHOOTER_LEAD_MOTOR_ID, SHOOTER_LEAD_TALONFX_CONFIG, "Shooter Lead"),
                new ShooterIOTalonFX(
                    SHOOTER_FOLLOW_MOTOR_ID,
                    SHOOTER_FOLLOW_TALONFX_CONFIG,
                    "Shooter Follow",
                    new Follower(SHOOTER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)),
                new ShooterIOTalonFX(
                    SHOOTER_KICKER_MOTOR_ID, SHOOTER_KICKER_TALONFX_CONFIG, "Shooter Kicker"),
                new ShooterIOTalonFX(
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
                new IntakeIOTalonFXSim(
                    INTAKE_ROLLER_MOTOR_ID,
                    INTAKE_ROLLER_TALONFX_CONFIG,
                    "KrakenX60_FOC",
                    "Intake Roller"),
                new IntakeIOTalonFXSim(
                    INTAKE_ARM_MOTOR_ID,
                    INTAKE_ARM_TALONFX_CONFIG,
                    "KrakenX60_FOC",
                    "Intake Arm",
                    Optional.of(INTAKE_ARM_CANCODER_CONFIG)));
        hopperSubsystem =
            new HopperSubsystem(
                new HopperIOTalonFXSim(
                    HOPPER_ROLLER_MOTOR_ID,
                    HOPPER_ROLLER_TALONFX_CONFIG,
                    "KrakenX60_FOC",
                    "Hopper Motor"));
        shooterSubsystem =
            new ShooterSubsystem(
                new ShooterIOTalonFXSim(
                    SHOOTER_LEAD_MOTOR_ID,
                    SHOOTER_LEAD_TALONFX_CONFIG,
                    "KrakenX60_FOC",
                    "Shooter Lead"),
                new ShooterIOTalonFXSim(
                    SHOOTER_FOLLOW_MOTOR_ID,
                    SHOOTER_FOLLOW_TALONFX_CONFIG,
                    "KrakenX60_FOC",
                    "Shooter Follow",
                    new Follower(SHOOTER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)),
                new ShooterIOTalonFXSim(
                    SHOOTER_KICKER_MOTOR_ID,
                    SHOOTER_KICKER_TALONFX_CONFIG,
                    "KrakenX60_FOC",
                    "Shooter Kicker"),
                new ShooterIOTalonFXSim(
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
                new IntakeIOTalonFXTunable(
                    INTAKE_ROLLER_MOTOR_ID, INTAKE_ROLLER_TALONFX_CONFIG, "Intake Roller"),
                new IntakeIOTalonFXTunable(
                    INTAKE_ARM_MOTOR_ID, INTAKE_ARM_TALONFX_CONFIG, "Intake Arm"));
        hopperSubsystem =
            new HopperSubsystem(
                new HopperIOTalonFXTunable(
                    HOPPER_ROLLER_MOTOR_ID, HOPPER_ROLLER_TALONFX_CONFIG, "Hopper Motor"));
        shooterSubsystem =
            new ShooterSubsystem(
                new ShooterIOTalonFXTunable(
                    SHOOTER_LEAD_MOTOR_ID, SHOOTER_LEAD_TALONFX_CONFIG, "Shooter Lead"),
                new ShooterIOTalonFXTunable(
                    SHOOTER_FOLLOW_MOTOR_ID,
                    SHOOTER_FOLLOW_TALONFX_CONFIG,
                    "Shooter Follow",
                    new Follower(SHOOTER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)),
                new ShooterIOTalonFXTunable(
                    SHOOTER_KICKER_MOTOR_ID, SHOOTER_KICKER_TALONFX_CONFIG, "Shooter Kicker"),
                new ShooterIOTalonFXTunable(
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
                new IntakeIOTalonFX(
                    INTAKE_ROLLER_MOTOR_ID, INTAKE_ROLLER_TALONFX_CONFIG, "Intake Roller"),
                new IntakeIOTalonFX(INTAKE_ARM_MOTOR_ID, INTAKE_ARM_TALONFX_CONFIG, "Intake Arm"));
        hopperSubsystem =
            new HopperSubsystem(
                new HopperIOTalonFX(
                    HOPPER_ROLLER_MOTOR_ID, HOPPER_ROLLER_TALONFX_CONFIG, "Hopper Motor"));
        shooterSubsystem =
            new ShooterSubsystem(
                new ShooterIOTalonFX(
                    SHOOTER_LEAD_MOTOR_ID, SHOOTER_LEAD_TALONFX_CONFIG, "Shooter Lead"),
                new ShooterIOTalonFX(
                    SHOOTER_FOLLOW_MOTOR_ID,
                    SHOOTER_FOLLOW_TALONFX_CONFIG,
                    "Shooter Follow",
                    new Follower(SHOOTER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)),
                new ShooterIOTalonFX(
                    SHOOTER_KICKER_MOTOR_ID, SHOOTER_KICKER_TALONFX_CONFIG, "Shooter Kicker"),
                new ShooterIOTalonFX(
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
    driverController.leftBumper()
        .whileTrue(intakeCommand)
        .whileTrue(indexToShooterCommand)
        .onFalse(deployIntakeCommand)
        .onFalse(stopHopperCommand);

    /* Outtake */
    driverController.rightBumper()
        .whileTrue(outtakeCommand)
        .whileTrue(indexToIntakeCommand)
        .onFalse(stopHopperCommand);

    /* Shoot */
    driverController.rightTrigger(0.2)
        .whileTrue(shootCommand)
        // .whileTrue(shootDriveCommand)
        .onFalse(stopShooterCommand)
        .onFalse(stopHopperCommand)
        .onFalse(stowIntakeCommand);

    /* Operator Controls */

    /* Manual Intake Arm */

    /* Manual Intake Roller */

    /* Manual Hopper Roller */

    /* Manual Shooter Kicker */

    /* Manual Shooter Roller */

    // Intake Arm
    operatorController
        .a()
        .whileTrue(
            new RunCommand(
                () -> intakeSubsystem.setIntakeArmSetpoint(INTAKE_ARM_DEPLOYED_POSITION),
                intakeSubsystem));

    operatorController
        .b()
        .whileTrue(
            new RunCommand(
                () -> intakeSubsystem.setIntakeArmSetpoint(INTAKE_ARM_STOWED_POSITION),
                intakeSubsystem));

    // Intake Roller
    operatorController
        .leftBumper()
        .whileTrue(
            new RunCommand(
                () -> intakeSubsystem.runIntakeRollerPercentage(intakeRollerSpeed),
                intakeSubsystem))
        .whileFalse(
            new RunCommand(
                () -> intakeSubsystem.runIntakeRollerPercentage(0),
                intakeSubsystem));
;

    // DogLog.tunable(
    //     "Intake Roller Speed %",
    //     intakeRollerSpeed, newSpeed -> intakeRollerSpeed = MathUtil.clamp(newSpeed, 0, 1));

    // Hopper Roller
    operatorController
        .leftStick()
        .whileTrue(
            new RunCommand(
                () -> hopperSubsystem.runHopperRollerPercentage(hopperRollerSpeed),
                hopperSubsystem))
        .whileFalse(
            new RunCommand(
                () -> hopperSubsystem.runHopperRollerPercentage(0),
                hopperSubsystem));
    // DogLog.tunable(
    //     "Hopper Roller Speed %",
    //     hopperRollerSpeed, newSpeed -> hopperRollerSpeed = MathUtil.clamp(newSpeed, 0, 1));

    // Shooter Kicker
    operatorController
        .leftTrigger(0.2)
        .whileTrue(
            new RunCommand(
                () -> shooterSubsystem.runKickerMotorPercentage(shooterKickerSpeed),
                shooterSubsystem))
        .whileFalse(
            new RunCommand(
                () -> shooterSubsystem.runKickerMotorPercentage(0),
                shooterSubsystem));
    // DogLog.tunable(
    //     "Shooter Kicker Speed %",
    //     shooterKickerSpeed, newSpeed -> shooterKickerSpeed = MathUtil.clamp(newSpeed, 0, 1));

    // Shooter Flywheels
    operatorController
        .rightTrigger(0.2)
        .whileTrue(
            new RunCommand(
                () -> shooterSubsystem.runShooterMotorPercentage(shooterLeadSpeed),
                shooterSubsystem))
        .whileFalse(            
            new RunCommand(
                () -> shooterSubsystem.runShooterMotorPercentage(0),
                shooterSubsystem));

    // DogLog.tunable(
    //     "Shooter Flywheel Speed %",
    //     shooterLeadSpeed, newSpeed -> shooterLeadSpeed = MathUtil.clamp(newSpeed, 0, 1));

    // // Intake to Hopper
    // driverController
    //     .leftBumper()
    //     .whileTrue(intakeCommand.alongWith(indexToShooterCommand))
    //     .whileFalse(deployIntakeCommand);

    // // Shoot FUEL //
    // driverController
    //     .rightTrigger(0.2)
    //     .whileTrue(indexToShooterCommand.alongWith(shootCommand))
    //     .whileFalse(stopHopperCommand.alongWith(stopShooterCommand));

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
