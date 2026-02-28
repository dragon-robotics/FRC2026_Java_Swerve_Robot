// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.constants.SwerveConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDriveCmd extends Command {

  private static class Speeds {
    double translation;
    double strafe;
    double rotation;
  }

  private final CommandSwerveDrivetrain swerve;
  private final DoubleSupplier translationSup;
  private final DoubleSupplier strafeSup;
  private final DoubleSupplier rotationSup;
  private final BooleanSupplier halfSpeedSup;

  private final Supplier<Optional<Rotation2d>> headingGetter;
  private final Consumer<Optional<Rotation2d>> headingSetter;
  private final DoubleSupplier rotationLastTriggeredGetter;
  private final DoubleConsumer rotationLastTriggeredSetter;

  private double maxSpeed;
  private double maxAngularRate;

  private final SwerveRequest.FieldCentric drive;
  private final SwerveRequest.FieldCentricFacingAngle driveMaintainHeading;

  /** Creates a new DefaultDriveCmd. */
  public DefaultDriveCmd(
      CommandSwerveDrivetrain swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier halfSpeedSup,
      Supplier<Optional<Rotation2d>> headingGetter,
      Consumer<Optional<Rotation2d>> headingSetter,
      DoubleSupplier rotationLastTriggeredGetter,
      DoubleConsumer rotationLastTriggeredSetter) {

    this.swerve = swerve;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.halfSpeedSup = halfSpeedSup;
    this.headingGetter = headingGetter;
    this.headingSetter = headingSetter;
    this.rotationLastTriggeredGetter = rotationLastTriggeredGetter;
    this.rotationLastTriggeredSetter = rotationLastTriggeredSetter;

    maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Max speed at 12 volts
    maxAngularRate =
        RotationsPerSecond.of(1).in(RadiansPerSecond); // 1 rotation per second max angular velocity

    drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.05)
            .withRotationalDeadband(maxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDesaturateWheelSpeeds(true);

    driveMaintainHeading =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.05)
            .withRotationalDeadband(maxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDesaturateWheelSpeeds(true);

    driveMaintainHeading.HeadingController.setPID(
        SwerveConstants.HEADING_KP, SwerveConstants.HEADING_KI, SwerveConstants.HEADING_KD);
    driveMaintainHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveMaintainHeading.HeadingController.setTolerance(SwerveConstants.HEADING_TOLERANCE);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rawRotation = rotationSup.getAsDouble();
    var speeds =
        processJoystickInputs(
            translationSup.getAsDouble(),
            strafeSup.getAsDouble(),
            rawRotation,
            halfSpeedSup.getAsBoolean());

    boolean rotationTriggered = Math.abs(rawRotation) > SwerveConstants.SWERVE_DEADBAND;
    boolean rotationActive =
        MathUtil.isNear(rotationLastTriggeredGetter.getAsDouble(), Timer.getFPGATimestamp(), 0.1)
            && (Math.abs(swerve.getState().Speeds.omegaRadiansPerSecond) > Math.toRadians(10));

    if (rotationTriggered) {
      rotationLastTriggeredSetter.accept(Timer.getFPGATimestamp());
    }

    if (rotationTriggered || rotationActive) {
      setSwerveToRotate(speeds.translation, speeds.strafe, speeds.rotation);
    } else {
      setSwerveToMaintainHeading(speeds.translation, speeds.strafe);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
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
    headingSetter.accept(Optional.empty()); // was: currentHeading = Optional.empty();
  }

  private void setSwerveToMaintainHeading(double translation, double strafe) {
    if (headingGetter.get().isEmpty()) { // was: currentHeading.isEmpty()
      headingSetter.accept(Optional.of(swerve.getState().Pose.getRotation()));
    }

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      Rotation2d targetDirection =
          alliance.get() == Alliance.Blue
              ? headingGetter.get().get() // was: currentHeading.get()
              : headingGetter.get().get().rotateBy(Rotation2d.fromDegrees(180));
      swerve.setControl(
          driveMaintainHeading
              .withVelocityX(translation)
              .withVelocityY(strafe)
              .withTargetDirection(targetDirection));
    }
  }
}
