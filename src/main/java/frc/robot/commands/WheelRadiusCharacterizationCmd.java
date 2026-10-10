package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.constants.SwerveConstants;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Measures effective wheel radius while slowly rotating on carpet. Adapted for Phoenix swerve from
 * the AdvantageKit wheel-radius characterization procedure:
 * https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/#wheel-radius-characterization
 */
public class WheelRadiusCharacterizationCmd extends Command {
  private static final String LOG_PREFIX = "Swerve/WheelRadiusCharacterization/";
  private static final double MODULE_ALIGNMENT_SECONDS = 1.0;
  private static final double MIN_WHEEL_DELTA_RADIANS = 1e-6;

  private final Supplier<SwerveDriveState> stateSupplier;
  private final Consumer<SwerveRequest> controlConsumer;
  private final double[] configuredWheelRadiiMeters;
  private final double[] startingDistancesMeters;
  private final double driveBaseRadiusMeters;
  private final Timer timer = new Timer();
  private final SlewRateLimiter rotationLimiter =
      new SlewRateLimiter(SwerveConstants.WHEEL_RADIUS_RAMP_RATE);
  private final SwerveRequest.RobotCentric rotationRequest =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.RobotCentric stopRequest =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

  private boolean measurementStarted;
  private Rotation2d previousHeading = Rotation2d.kZero;
  private double gyroDeltaRadians;
  private double wheelDeltaRadians;

  public WheelRadiusCharacterizationCmd(
      CommandSwerveDrivetrain drivetrain, SwerveModuleConstants<?, ?, ?>... modules) {
    this(drivetrain, drivetrain::getStateCopy, drivetrain::setControl, modules);
  }

  WheelRadiusCharacterizationCmd(
      Subsystem drivetrain,
      Supplier<SwerveDriveState> stateSupplier,
      Consumer<SwerveRequest> controlConsumer,
      SwerveModuleConstants<?, ?, ?>... modules) {
    this.stateSupplier = stateSupplier;
    this.controlConsumer = controlConsumer;
    configuredWheelRadiiMeters = new double[modules.length];
    startingDistancesMeters = new double[modules.length];
    double totalModuleRadiusMeters = 0.0;
    for (int i = 0; i < modules.length; i++) {
      configuredWheelRadiiMeters[i] = modules[i].WheelRadius;
      totalModuleRadiusMeters += Math.hypot(modules[i].LocationX, modules[i].LocationY);
    }
    driveBaseRadiusMeters = totalModuleRadiusMeters / modules.length;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    timer.restart();
    rotationLimiter.reset(0.0);
    measurementStarted = false;
    gyroDeltaRadians = 0.0;
    wheelDeltaRadians = 0.0;
    DogLog.forceNt.log(LOG_PREFIX + "Running", true);
    logMeasurements();
  }

  @Override
  public void execute() {
    controlConsumer.accept(
        rotationRequest.withRotationalRate(
            rotationLimiter.calculate(SwerveConstants.WHEEL_RADIUS_MAX_VELOCITY)));
    if (!timer.hasElapsed(MODULE_ALIGNMENT_SECONDS)) {
      return;
    }

    SwerveDriveState state = stateSupplier.get();
    if (!measurementStarted) {
      for (int i = 0; i < startingDistancesMeters.length; i++) {
        startingDistancesMeters[i] = state.ModulePositions[i].distanceMeters;
      }
      previousHeading = state.RawHeading;
      measurementStarted = true;
    } else {
      updateMeasurements(state);
    }
    logMeasurements();
  }

  @Override
  public void end(boolean interrupted) {
    // Explicitly replace the rotation target; Phoenix Idle leaves motor commands unchanged.
    controlConsumer.accept(stopRequest);
    timer.stop();
    if (measurementStarted) {
      updateMeasurements(stateSupplier.get());
    }
    logMeasurements();
    DogLog.forceNt.log(LOG_PREFIX + "Running", false);
  }

  private void updateMeasurements(SwerveDriveState state) {
    gyroDeltaRadians += Math.abs(state.RawHeading.minus(previousHeading).getRadians());
    previousHeading = state.RawHeading;

    wheelDeltaRadians = 0.0;
    for (int i = 0; i < startingDistancesMeters.length; i++) {
      // Phoenix module distance already accounts for gearing and steer/drive coupling.
      // Dividing out its configured radius recovers wheel radians without assuming that
      // the configured radius matches the effective radius being measured.
      wheelDeltaRadians +=
          Math.abs(state.ModulePositions[i].distanceMeters - startingDistancesMeters[i])
              / configuredWheelRadiiMeters[i];
    }
    wheelDeltaRadians /= startingDistancesMeters.length;
  }

  private void logMeasurements() {
    double radiusMeters = 0.0;
    if (wheelDeltaRadians > MIN_WHEEL_DELTA_RADIANS) {
      double measuredRadius = gyroDeltaRadians * driveBaseRadiusMeters / wheelDeltaRadians;
      if (Double.isFinite(measuredRadius)) {
        radiusMeters = measuredRadius;
      }
    }
    boolean valid = gyroDeltaRadians >= 2.0 * Math.PI && radiusMeters > 0.0;
    DogLog.forceNt.log(LOG_PREFIX + "GyroDeltaRadians", gyroDeltaRadians);
    DogLog.forceNt.log(LOG_PREFIX + "WheelDeltaRadians", wheelDeltaRadians);
    DogLog.forceNt.log(LOG_PREFIX + "RadiusMeters", radiusMeters);
    DogLog.forceNt.log(LOG_PREFIX + "RadiusInches", Units.metersToInches(radiusMeters));
    DogLog.forceNt.log(LOG_PREFIX + "Valid", valid);
  }
}
