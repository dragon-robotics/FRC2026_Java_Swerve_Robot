package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIO.MotorIOInputs;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShooterSubsystemTest {

  private static boolean halReady;

  @BeforeAll
  static void initializeHal() {
    halReady = HAL.initialize(500, 0);
  }

  @BeforeEach
  void setTeleopMode() {
    Assumptions.assumeTrue(halReady, "HAL/simulation unavailable in this environment");
    setAutonomousEnabled(false);
  }

  @AfterEach
  void resetDriverStation() {
    if (halReady) {
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    }
  }

  @AfterAll
  static void shutdownHal() {
    if (halReady) {
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
      HAL.shutdown();
    }
  }

  @Test
  void constructorZerosHoodEncoderToKnownStartupPose() {
    FakeMotorIO hood = new FakeMotorIO();

    new ShooterSubsystem(new FakeMotorIO(), new FakeMotorIO(), new FakeMotorIO(), hood);

    assertNotNull(hood.lastResetPosition);
    assertEquals(0.0, hood.lastResetPosition, 1e-9);
  }

  @Test
  void constructorDoesNotResetFlywheelOrKickerEncoders() {
    FakeMotorIO lead = new FakeMotorIO();
    FakeMotorIO follow = new FakeMotorIO();
    FakeMotorIO kicker = new FakeMotorIO();

    new ShooterSubsystem(lead, follow, kicker, new FakeMotorIO());

    assertNull(lead.lastResetPosition);
    assertNull(follow.lastResetPosition);
    assertNull(kicker.lastResetPosition);
  }

  @Test
  void autonomousPrepCommands2700RpmAndWaitsForAutoReadiness() {
    FakeMotorIO lead = new FakeMotorIO();
    FakeMotorIO kicker = new FakeMotorIO();
    lead.velocityRpm = 2000.0;
    ShooterSubsystem shooter =
        new ShooterSubsystem(lead, new FakeMotorIO(), kicker, new FakeMotorIO());
    setAutonomousEnabled(true);
    shooter.setDesiredState(ShooterState.PREPFUEL);

    shooter.periodic();
    assertEquals(2700.0, lead.lastRpm, 1e-9);
    assertNotNull(kicker.lastVoltage);
    assertSame(ShooterState.TRANSITION, shooter.getCurrentState());

    lead.velocityRpm = 2700.0;
    shooter.periodic();
    assertSame(ShooterState.PREPFUEL, shooter.getCurrentState());
  }

  @Test
  void teleopPrepCommands2000RpmAndUsesTeleopReadiness() {
    FakeMotorIO lead = new FakeMotorIO();
    lead.velocityRpm = 2700.0;
    ShooterSubsystem shooter =
        new ShooterSubsystem(lead, new FakeMotorIO(), new FakeMotorIO(), new FakeMotorIO());
    shooter.setDesiredState(ShooterState.PREPFUEL);

    shooter.periodic();
    assertEquals(2000.0, lead.lastRpm, 1e-9);
    assertSame(ShooterState.TRANSITION, shooter.getCurrentState());

    lead.velocityRpm = 2000.0;
    shooter.periodic();
    assertSame(ShooterState.PREPFUEL, shooter.getCurrentState());
  }

  private static class FakeMotorIO implements MotorIO {
    private Double lastResetPosition;
    private Double lastRpm;
    private Double lastPosition;
    private Voltage lastVoltage;
    private double velocityRpm;

    @Override
    public void setMotorVoltage(Voltage voltage) {
      lastVoltage = voltage;
    }

    @Override
    public void setMotorRPM(double rpm) {
      lastRpm = rpm;
    }

    @Override
    public void setMotorPosition(double position) {
      lastPosition = position;
    }

    public void resetMotorPosition(double position) {
      lastResetPosition = position;
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
      inputs.setMotorVelocity(velocityRpm / 60.0);
    }
  }

  private static void setAutonomousEnabled(boolean enabled) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(enabled);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }
}
