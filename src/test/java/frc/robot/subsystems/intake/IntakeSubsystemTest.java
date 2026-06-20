package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeSubsystem.IntakeState.DEPLOYED;
import static frc.robot.subsystems.intake.IntakeSubsystem.IntakeState.INTAKE;
import static frc.robot.subsystems.intake.IntakeSubsystem.IntakeState.OUTTAKE;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_DEPLOYED_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_STOWED_POSITION;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIO.MotorIOInputs;
import frc.robot.io.TorqueCurrentMotorIO;
import org.junit.jupiter.api.Test;

class IntakeSubsystemTest {

  @Test
  void intakeRequestAppliesTensionOnlyAfterArmReachesDeployedPosition() {
    FakeMotorIO rollerLead = new FakeMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeTorqueCurrentMotorIO arm = new FakeTorqueCurrentMotorIO();
    arm.position = INTAKE_ARM_STOWED_POSITION;

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);
    intake.periodic();

    intake.setDesiredState(INTAKE);
    intake.periodic();

    assertNull(arm.lastTorqueCurrent);

    arm.position = INTAKE_ARM_DEPLOYED_POSITION;
    intake.periodic();
    assertNull(arm.lastTorqueCurrent);

    intake.periodic();

    assertNotNull(arm.lastTorqueCurrent);
    assertEquals(20.0, arm.lastTorqueCurrent.in(edu.wpi.first.units.Units.Amps), 1e-9);
  }

  @Test
  void deployedRequestHoldsPositionWithoutTorqueCurrent() {
    FakeMotorIO rollerLead = new FakeMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeTorqueCurrentMotorIO arm = new FakeTorqueCurrentMotorIO();
    arm.position = INTAKE_ARM_DEPLOYED_POSITION;

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);
    intake.periodic();
    arm.lastPositionSetpoint = null;

    intake.setDesiredState(DEPLOYED);

    assertDoesNotThrow(intake::periodic);
    assertEquals(INTAKE_ARM_DEPLOYED_POSITION, arm.lastPositionSetpoint, 1e-9);
    assertNull(arm.lastTorqueCurrent);
  }

  @Test
  void intakeRequestFallsBackToPositionHoldWhenArmCannotTorqueCurrent() {
    FakeMotorIO rollerLead = new FakeMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeMotorIO arm = new FakeMotorIO();
    arm.position = INTAKE_ARM_DEPLOYED_POSITION;

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);
    intake.periodic();
    arm.lastPositionSetpoint = null;

    intake.setDesiredState(INTAKE);

    assertDoesNotThrow(intake::periodic);
    assertEquals(INTAKE_ARM_DEPLOYED_POSITION, arm.lastPositionSetpoint, 1e-9);
  }

  @Test
  void outtakeRequestHoldsPositionWithoutTorqueCurrent() {
    FakeMotorIO rollerLead = new FakeMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeTorqueCurrentMotorIO arm = new FakeTorqueCurrentMotorIO();
    arm.position = INTAKE_ARM_DEPLOYED_POSITION;

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);
    intake.periodic();
    arm.lastPositionSetpoint = null;

    intake.setDesiredState(OUTTAKE);

    assertDoesNotThrow(intake::periodic);
    assertEquals(INTAKE_ARM_DEPLOYED_POSITION, arm.lastPositionSetpoint, 1e-9);
    assertNull(arm.lastTorqueCurrent);
  }

  private static class FakeMotorIO implements MotorIO {
    protected double position;
    protected Double lastPositionSetpoint;

    @Override
    public void setMotorVoltage(Voltage voltage) {}

    @Override
    public void setMotorPosition(double setpoint, int slotID) {
      lastPositionSetpoint = setpoint;
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
      inputs.setMotorPosition(position);
    }
  }

  private static class FakeTorqueCurrentMotorIO extends FakeMotorIO
      implements TorqueCurrentMotorIO {
    private Current lastTorqueCurrent;

    @Override
    public void setMotorTorqueCurrent(Current torqueCurrent) {
      lastTorqueCurrent = torqueCurrent;
    }
  }
}
