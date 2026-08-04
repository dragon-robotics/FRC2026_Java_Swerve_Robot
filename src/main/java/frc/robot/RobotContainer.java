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
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_FOLLOW_MOTOR_ID;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_FOLLOW_TALONFX_CONFIG;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_LEAD_MOTOR_ID;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_LEAD_TALONFX_CONFIG;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
import frc.robot.subsystems.Superstructure.ShootMode;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.intake.IntakeSubsystemSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.OperatorConstants;
import frc.robot.util.constants.SwerveConstants;
import frc.robot.util.constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Central wiring point for robot subsystems, command factories, controls, and
 * dashboards.
 */
public class RobotContainer {

  /* Robot Subsystems */
  public final CommandSwerveDrivetrain swerveSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  public final HopperSubsystem hopperSubsystem;
  public final ShooterSubsystem shooterSubsystem;
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
  private Command shootNoAimCommand;

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  private final Field2d field = new Field2d();
  private static final Pose2d HIDDEN_VISION_POSE = new Pose2d(-10.0, -10.0, new Rotation2d());
  // Most FRC views expose fewer than 8 tags at once; cap keeps NT topic count
  // stable.
  private static final int MAX_VISION_TAG_LINE_OBJECTS = 8;
  private static final double VISION_TAG_LINE_HOLD_SECONDS = 0.2;
  private final List<Pose2d[]> lastVisionTagLineSegments = new ArrayList<>();
  private double lastVisionTagUpdateTimestamp = -1.0;
  private Supplier<Pose2d> visionSimulationPoseSupplier;

  private record MechanismSubsystems(
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      ShooterSubsystem shooter,
      VisionSubsystem vision) {
  }

  public RobotContainer() {

    configureDashboardAndLogging();

    driverController = new CommandXboxController(OperatorConstants.DRIVER_PORT);
    operatorController = new CommandXboxController(OperatorConstants.OPERATOR_PORT);

    swerveSubsystem = createSwerveSubsystem();
    visionSimulationPoseSupplier = () -> swerveSubsystem.getState().Pose;
    MechanismSubsystems mechanisms = createMechanismSubsystems();
    intakeSubsystem = mechanisms.intake();
    hopperSubsystem = mechanisms.hopper();
    shooterSubsystem = mechanisms.shooter();
    visionSubsystem = mechanisms.vision();

    superstructureSubsystem = createSuperstructure();
    configureSuperstructureCommands();
    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
    warmupPathPlanner();
    configureHubShiftOverride();
  }

  private void configureDashboardAndLogging() {
    SmartDashboard.putData("Field", field);
    DogLog.setOptions(new DogLogOptions());
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private CommandSwerveDrivetrain createSwerveSubsystem() {
    return TunerConstants.createDrivetrain(
        250, SwerveConstants.ODOMETRY_STD, VisionConstants.DEFAULT_TAG_STDDEV);
  }

  private MechanismSubsystems createMechanismSubsystems() {
    return switch (CURRENT_MODE) {
      case COMP -> createCompetitionSubsystems();
      case SIM -> createSimulationSubsystems();
      case TEST -> createTunableTestSubsystems();
      default -> createCompetitionSubsystems();
    };
  }

  private Superstructure createSuperstructure() {
    return new Superstructure(
        swerveSubsystem, intakeSubsystem, hopperSubsystem, shooterSubsystem, visionSubsystem, this);
  }

  private void configureSuperstructureCommands() {
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
    intakeCommand = superstructureSubsystem.setStateCmd(SuperState.INTAKE);
    shootCommand = superstructureSubsystem.shootWithJuicerDelayCmd();
    shootNoAimCommand = superstructureSubsystem.shootNoAimWithJuicerDelayCmd();
    driveCommand = superstructureSubsystem.setStateCmd(SuperState.DRIVE);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Intake", intakeCommand);
    NamedCommands.registerCommand("Shoot", shootCommand);
    NamedCommands.registerCommand("ShootNoAim", shootNoAimCommand);
    NamedCommands.registerCommand("Drive", driveCommand);
  }

  private void warmupPathPlanner() {
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  private void configureHubShiftOverride() {
    SmartDashboard.putString("HubShift/WonAuto", "");
    HubShiftUtil.setAllianceWinOverride(
        () -> {
          String override = SmartDashboard.getString("HubShift/WonAuto", "");
          if (override.equalsIgnoreCase("true")) {
            return Optional.of(true);
          }
          if (override.equalsIgnoreCase("false")) {
            return Optional.of(false);
          }
          return Optional.empty();
        });
  }

  private MechanismSubsystems createCompetitionSubsystems() {
    IntakeSubsystem intake = new IntakeSubsystem(
        new TalonFXMotorIO(
            INTAKE_ROLLER_LEAD_MOTOR_ID,
            INTAKE_ROLLER_LEAD_TALONFX_CONFIG,
            "Intake Roller Lead"),
        new TalonFXMotorIO(
            INTAKE_ROLLER_FOLLOW_MOTOR_ID,
            INTAKE_ROLLER_FOLLOW_TALONFX_CONFIG,
            "Intake Roller Follow",
            new Follower(INTAKE_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)),
        new TalonFXMotorIO(INTAKE_ARM_MOTOR_ID, INTAKE_ARM_TALONFX_CONFIG, "Intake Arm"));
    HopperSubsystem hopper = new HopperSubsystem(
        new TalonFXMotorIO(
            HOPPER_ROLLER_LEAD_MOTOR_ID,
            HOPPER_ROLLER_LEAD_TALONFX_CONFIG,
            "Hopper Lead Motor"),
        new TalonFXMotorIO(
            HOPPER_ROLLER_FOLLOW_MOTOR_ID,
            HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG,
            "Hopper Follow Motor",
            new Follower(HOPPER_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Aligned)));
    ShooterSubsystem shooter = new ShooterSubsystem(
        new TalonFXMotorIO(SHOOTER_LEAD_MOTOR_ID, SHOOTER_LEAD_TALONFX_CONFIG, "Shooter Lead"),
        new TalonFXMotorIO(
            SHOOTER_FOLLOW_MOTOR_ID,
            SHOOTER_FOLLOW_TALONFX_CONFIG,
            "Shooter Follow",
            new Follower(SHOOTER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)),
        new TalonFXMotorIO(
            SHOOTER_KICKER_MOTOR_ID, SHOOTER_KICKER_TALONFX_CONFIG, "Shooter Kicker"),
        new TalonFXMotorIO(SHOOTER_HOOD_MOTOR_ID, SHOOTER_HOOD_TALONFX_CONFIG, "Shooter Hood"));

    return new MechanismSubsystems(intake, hopper, shooter, createRealVisionSubsystem());
  }

  private MechanismSubsystems createSimulationSubsystems() {
    IntakeSubsystem intake = new IntakeSubsystemSim(
        new TalonFXMotorIOSim(
            INTAKE_ROLLER_LEAD_MOTOR_ID,
            INTAKE_ROLLER_LEAD_TALONFX_CONFIG,
            "KrakenX60",
            "Intake Roller Lead"),
        new TalonFXMotorIOSim(
            INTAKE_ROLLER_FOLLOW_MOTOR_ID,
            INTAKE_ROLLER_FOLLOW_TALONFX_CONFIG,
            "KrakenX60",
            "Intake Roller Follow",
            new Follower(INTAKE_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)),
        new TalonFXMotorIOSim(
            INTAKE_ARM_MOTOR_ID,
            INTAKE_ARM_TALONFX_CONFIG,
            "KrakenX60",
            "Intake Arm",
            INTAKE_ARM_CANCODER_CONFIG));
    HopperSubsystem hopper = new HopperSubsystem(
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
    ShooterSubsystem shooter = new ShooterSubsystem(
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
            SHOOTER_HOOD_MOTOR_ID, SHOOTER_HOOD_TALONFX_CONFIG, "KrakenX44", "Shooter Hood"));

    return new MechanismSubsystems(intake, hopper, shooter, createSimulationVisionSubsystem());
  }

  private MechanismSubsystems createTunableTestSubsystems() {
    IntakeSubsystem intake = new IntakeSubsystem(
        new TalonFXMotorIOTunable(
            INTAKE_ROLLER_LEAD_MOTOR_ID,
            INTAKE_ROLLER_LEAD_TALONFX_CONFIG,
            "Intake Roller Lead"),
        new TalonFXMotorIOTunable(
            INTAKE_ROLLER_FOLLOW_MOTOR_ID,
            INTAKE_ROLLER_FOLLOW_TALONFX_CONFIG,
            "Intake Roller Follow",
            new Follower(INTAKE_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Opposed)),
        new TalonFXMotorIOTunable(
            INTAKE_ARM_MOTOR_ID, INTAKE_ARM_TALONFX_CONFIG, "Intake Arm"));
    HopperSubsystem hopper = new HopperSubsystem(
        new TalonFXMotorIOTunable(
            HOPPER_ROLLER_LEAD_MOTOR_ID,
            HOPPER_ROLLER_LEAD_TALONFX_CONFIG,
            "Hopper Lead Motor"),
        new TalonFXMotorIOTunable(
            HOPPER_ROLLER_FOLLOW_MOTOR_ID,
            HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG,
            "Hopper Follow Motor",
            new Follower(HOPPER_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Aligned)));
    ShooterSubsystem shooter = new ShooterSubsystem(
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

    return new MechanismSubsystems(intake, hopper, shooter, createRealVisionSubsystem());
  }

  private VisionSubsystem createRealVisionSubsystem() {
    return new VisionSubsystem(swerveSubsystem, swerveSubsystem::addVisionMeasurement);
    // return new VisionSubsystem(
    // swerveSubsystem,
    // swerveSubsystem::addVisionMeasurement,
    // new VisionIOPhotonVision(APTAG_CAMERA_NAMES[0],
    // VisionConstants.APTAG_POSE_EST_CAM_F_POS),
    // new VisionIOPhotonVision(APTAG_CAMERA_NAMES[1],
    // VisionConstants.APTAG_POSE_EST_CAM_R_POS),
    // new VisionIOPhotonVision(APTAG_CAMERA_NAMES[2],
    // VisionConstants.APTAG_POSE_EST_CAM_B_POS),
    // new VisionIOPhotonVision(APTAG_CAMERA_NAMES[3],
    // VisionConstants.APTAG_POSE_EST_CAM_L_POS));
  }

  private VisionSubsystem createSimulationVisionSubsystem() {
    return new VisionSubsystem(
        swerveSubsystem,
        swerveSubsystem::addVisionMeasurement,
        new VisionIOPhotonVisionSim(
            APTAG_CAMERA_NAMES[0],
            VisionConstants.APTAG_POSE_EST_CAM_F_POS,
            this::getVisionSimulationPose),
        new VisionIOPhotonVisionSim(
            APTAG_CAMERA_NAMES[1],
            VisionConstants.APTAG_POSE_EST_CAM_R_POS,
            this::getVisionSimulationPose),
        new VisionIOPhotonVisionSim(
            APTAG_CAMERA_NAMES[2],
            VisionConstants.APTAG_POSE_EST_CAM_B_POS,
            this::getVisionSimulationPose),
        new VisionIOPhotonVisionSim(
            APTAG_CAMERA_NAMES[3],
            VisionConstants.APTAG_POSE_EST_CAM_L_POS,
            this::getVisionSimulationPose));
  }

  private Pose2d getVisionSimulationPose() {
    return visionSimulationPoseSupplier.get();
  }

  /**
   * Overrides the field-relative robot pose used by simulated PhotonVision
   * cameras.
   *
   * <p>
   * Real robot code does not use this hook. Vision regression tests set it to
   * their independent
   * ground-truth pose so simulated detections are not sourced from the drivetrain
   * estimator they
   * are validating.
   */
  public void setVisionSimulationPoseSupplier(Supplier<Pose2d> poseSupplier) {
    visionSimulationPoseSupplier = poseSupplier;
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
    var shootTrigger = driverController.rightTrigger(0.2);

    // Purge path owns intake because it actively outtakes while shooting.
    shootTrigger
        .and(superstructureSubsystem::shouldUsePurgeDuringShoot)
        .whileTrue(
            Commands.defer(
                superstructureSubsystem::purgeShootCmd,
                Set.of(swerveSubsystem, intakeSubsystem, hopperSubsystem, shooterSubsystem)));

    // Non-purge shooting leaves intake free for independent JUICER override.
    shootTrigger
        .and(() -> !superstructureSubsystem.shouldUsePurgeDuringShoot())
        .whileTrue(
            Commands.defer(
                superstructureSubsystem::selectedShootModeCmd,
                Set.of(swerveSubsystem, hopperSubsystem, shooterSubsystem)));

    shootTrigger.onFalse(superstructureSubsystem.setStateCmd(SuperState.DRIVE));

    /* Driver can control the Juicer mode if needed */
    driverController
        .b()
        .whileTrue(superstructureSubsystem.intakeOverrideCmd(IntakeState.JUICER))
        .onFalse(superstructureSubsystem.intakeOverrideCmd(IntakeState.DEPLOYED));

    driverController
        .a()
        .whileTrue(superstructureSubsystem.setStateCmd(SuperState.DRIVE_STARTING_CONFIG));

    /* Operator Controls */

    /* Reset the robot pose based on vision — operator back and start button */
    operatorController
        .start()
        .and(operatorController.back())
        .onTrue(superstructureSubsystem.forceReseedFromVisionCmd());

    /* Operator mainly controls the Juicer mode */
    operatorController
        .b()
        .and(operatorController.a().negate())
        .whileTrue(superstructureSubsystem.intakeOverrideCmd(IntakeState.JUICER))
        .onFalse(superstructureSubsystem.intakeOverrideCmd(IntakeState.DEPLOYED));

    /* Manual shooter setpoint toggles */
    operatorController
        .x()
        .and(operatorController.a())
        .onTrue(superstructureSubsystem.toggleShootModeCmd(ShootMode.MANUAL_BUMPER_UP));

    operatorController
        .a()
        .and(operatorController.b())
        .onTrue(superstructureSubsystem.toggleShootModeCmd(ShootMode.MANUAL_TRENCH));

    operatorController
        .rightBumper()
        .whileTrue(
            Commands.run(() -> shooterSubsystem.runKickerMotorPercentage(1), shooterSubsystem)
                .withName("Kicker Full Power"));
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  /**
   * Updates the SmartDashboard Field2d topic with drivetrain and accepted vision
   * poses.
   */
  public void updateFieldDashboard() {
    Pose2d swervePose = swerveSubsystem.getState().Pose;
    field.setRobotPose(swervePose);

    Optional<VisionSubsystem.AcceptedObservationSnapshot> acceptedSnapshotOpt = visionSubsystem
        .getLatestAcceptedObservationSnapshot();
    updateAcceptedVisionPose(acceptedSnapshotOpt);
    updateVisionTagLines(acceptedSnapshotOpt);
  }

  private void updateAcceptedVisionPose(
      Optional<VisionSubsystem.AcceptedObservationSnapshot> acceptedSnapshotOpt) {
    if (acceptedSnapshotOpt.isPresent()) {
      field.getObject("VisionAcceptedPose").setPose(acceptedSnapshotOpt.get().pose());
    } else {
      field.getObject("VisionAcceptedPose").setPose(HIDDEN_VISION_POSE);
    }
  }

  private void updateVisionTagLines(
      Optional<VisionSubsystem.AcceptedObservationSnapshot> acceptedSnapshotOpt) {
    if (acceptedSnapshotOpt.isPresent()) {
      cacheVisionTagLineSegments(acceptedSnapshotOpt.get());
    }

    renderVisionTagLineSegments(shouldRenderHeldVisionTagSegments());
  }

  private void cacheVisionTagLineSegments(VisionSubsystem.AcceptedObservationSnapshot snapshot) {
    List<Pose2d[]> newSegments = new ArrayList<>();
    for (int tagId : snapshot.tagIDs()) {
      FieldConstants.APTAG_FIELD_LAYOUT
          .getTagPose(tagId)
          .ifPresent(
              tagPose -> newSegments.add(new Pose2d[] { snapshot.pose(), tagPose.toPose2d() }));
    }
    if (!newSegments.isEmpty()) {
      lastVisionTagLineSegments.clear();
      lastVisionTagLineSegments.addAll(newSegments);
      lastVisionTagUpdateTimestamp = Timer.getFPGATimestamp();
    }
  }

  private boolean shouldRenderHeldVisionTagSegments() {
    return !lastVisionTagLineSegments.isEmpty()
        && (Timer.getFPGATimestamp() - lastVisionTagUpdateTimestamp) <= VISION_TAG_LINE_HOLD_SECONDS;
  }

  private void renderVisionTagLineSegments(boolean shouldRenderHeldSegments) {
    for (int i = 0; i < MAX_VISION_TAG_LINE_OBJECTS; i++) {
      var tagLineObject = field.getObject("VisionTagLine_" + i + "Trajectory");
      if (shouldRenderHeldSegments && i < lastVisionTagLineSegments.size()) {
        Pose2d[] segment = lastVisionTagLineSegments.get(i);
        tagLineObject.setTrajectory(createLineTrajectory(segment[0], segment[1]));
      } else {
        tagLineObject.setTrajectory(new Trajectory());
      }
    }
  }

  private static Trajectory createLineTrajectory(Pose2d startPose, Pose2d endPose) {
    return new Trajectory(
        List.of(
            new Trajectory.State(0.0, 0.0, 0.0, startPose, 0.0),
            // Use a longer synthetic duration so Field2d sampling yields multiple points.
            new Trajectory.State(1.0, 0.0, 0.0, endPose, 0.0)));
  }
}
