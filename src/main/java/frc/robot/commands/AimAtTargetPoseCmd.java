// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.SwerveConstants;
import java.util.Optional;
import java.util.function.Consumer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtTargetPoseCmd extends Command {

  private CommandSwerveDrivetrain swerve;
  private Consumer<Optional<Rotation2d>> setCurrentHeading;

  private double maxSpeed;
  private double maxAngularRate;

  private final SwerveRequest.FieldCentricFacingAngle driveMaintainHeading;

  /** Creates a new AimAtTargetPoseCmd. */
  public AimAtTargetPoseCmd(
      CommandSwerveDrivetrain swerve, Consumer<Optional<Rotation2d>> setCurrentHeading) {
    this.swerve = swerve;
    this.setCurrentHeading = setCurrentHeading;

    driveMaintainHeading =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.05)
            .withRotationalDeadband(maxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDesaturateWheelSpeeds(true);

    /* Set the PID constants for the Maintain Heading controller */
    driveMaintainHeading.HeadingController.setPID(
        SwerveConstants.HEADING_KP, SwerveConstants.HEADING_KI, SwerveConstants.HEADING_KD);
    driveMaintainHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveMaintainHeading.HeadingController.setTolerance(SwerveConstants.HEADING_TOLERANCE);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Grab the robot's current alliance
    Optional<Alliance> alliance = DriverStation.getAlliance();

    // Choose which hub to aim to depending on alliance color
    Translation2d hubToAimTowards =
        alliance.isPresent() && (alliance.get() == Alliance.Red)
            ? FieldConstants.Hub.RED_CENTER_POSE
            : FieldConstants.Hub.BLUE_CENTER_POSE;

    // Current robot translation (x,y) in field coordinates
    var robotTranslation = swerve.getState().Pose.getTranslation();

    // Vector from robot to target hub
    var delta = hubToAimTowards.minus(robotTranslation);

    // Absolute angle in field frame — hubToAimTowards is already alliance-specific,
    // so atan2 gives the correct direction-to-hub angle for both alliances.
    Rotation2d angleToPointAt = new Rotation2d(Math.atan2(delta.getY(), delta.getX()));
    Rotation2d targetHeading = toOperatorPerspectiveHeading(angleToPointAt);
    setCurrentHeading.accept(Optional.of(targetHeading));

    swerve.setControl(
        driveMaintainHeading.withTargetDirection(targetHeading).withTargetRateFeedforward(0.1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveMaintainHeading.HeadingController.atSetpoint();
  }

  private Rotation2d toOperatorPerspectiveHeading(Rotation2d fieldHeading) {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? fieldHeading.rotateBy(Rotation2d.kPi)
        : fieldHeading;
  }
}
