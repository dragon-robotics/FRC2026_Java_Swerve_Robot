package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class DefaultDriveCmdTest {

  @Test
  void demoJoystickInputHalvesFieldCentricTranslationStrafeAndRotation() {
    CapturingSwerve swerve = new CapturingSwerve();
    DefaultDriveCmd command =
        new DefaultDriveCmd(
            swerve,
            () -> 1.0,
            () -> -1.0,
            () -> 1.0,
            () -> false,
            () -> false,
            Optional::empty,
            ignored -> {},
            () -> 0.0,
            ignored -> {},
            Optional::empty);

    command.execute();

    assertNotNull(swerve.lastRequest);
    SwerveRequest.FieldCentric request =
        assertInstanceOf(SwerveRequest.FieldCentric.class, swerve.lastRequest);
    assertAll(
        () ->
            assertEquals(
                TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.5,
                request.VelocityX,
                1e-9),
        () ->
            assertEquals(
                -TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.5,
                request.VelocityY,
                1e-9),
        () -> assertEquals(Math.PI, request.RotationalRate, 1e-9));
  }

  private static class CapturingSwerve extends CommandSwerveDrivetrain {
    private SwerveRequest lastRequest;

    CapturingSwerve() {
      super(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);
    }

    @Override
    public void setControl(SwerveRequest request) {
      lastRequest = request;
    }
  }
}
