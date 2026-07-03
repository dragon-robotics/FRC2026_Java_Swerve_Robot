package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import frc.robot.io.MotorIO;
import org.junit.jupiter.api.Test;

class ShooterSubsystemTest {

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

  private static class FakeMotorIO implements MotorIO {
    private Double lastResetPosition;

    public void resetMotorPosition(double position) {
      lastResetPosition = position;
    }
  }
}
