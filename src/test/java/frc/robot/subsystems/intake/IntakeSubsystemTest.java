package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeSubsystem.IntakeState.DEPLOYED;
import static frc.robot.subsystems.intake.IntakeSubsystem.IntakeState.INTAKE;
import static frc.robot.subsystems.intake.IntakeSubsystem.IntakeState.JUICER;
import static frc.robot.subsystems.intake.IntakeSubsystem.IntakeState.OUTTAKE;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_DEPLOYED_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ARM_STOWED_POSITION;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_FOLLOW_TALONFX_CONFIG;
import static frc.robot.util.constants.IntakeConstants.INTAKE_ROLLER_LEAD_TALONFX_CONFIG;
import static org.junit.jupiter.api.Assertions.assertAll;
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
  void counterRotatingRollersUseMatchingControllerInversion() {
    assertEquals(
        INTAKE_ROLLER_LEAD_TALONFX_CONFIG.MotorOutput.Inverted,
        INTAKE_ROLLER_FOLLOW_TALONFX_CONFIG.MotorOutput.Inverted);
  }

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
    assertEquals(-10.0, arm.lastTorqueCurrent.in(edu.wpi.first.units.Units.Amps), 1e-9);
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

  @Test
  void juicerRunsRollerWithTorqueCurrentControl() {
    FakeTorqueCurrentMotorIO rollerLead = new FakeTorqueCurrentMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeMotorIO arm = new FakeMotorIO();

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);

    intake.setDesiredState(JUICER);
    intake.periodic();

    assertAll(
        () -> assertNotNull(rollerLead.lastTorqueCurrent),
        () -> assertEquals(90.0, rollerLead.lastTorqueCurrent.in(Amps), 1e-9),
        () -> assertNotNull(rollerLead.lastMaxAbsDutyCycle),
        () -> assertEquals(0.5, rollerLead.lastMaxAbsDutyCycle, 1e-9),
        () -> assertNull(rollerLead.lastVoltage));
  }

  @Test
  void runIntakeCommandsRollerTorqueCurrentWithDutyCycleCap() {
    FakeTorqueCurrentMotorIO rollerLead = new FakeTorqueCurrentMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeMotorIO arm = new FakeMotorIO();

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);

    intake.runIntake();

    assertAll(
        () -> assertNotNull(rollerLead.lastTorqueCurrent),
        () -> assertEquals(90.0, rollerLead.lastTorqueCurrent.in(Amps), 1e-9),
        () -> assertNotNull(rollerLead.lastMaxAbsDutyCycle),
        () -> assertEquals(0.8, rollerLead.lastMaxAbsDutyCycle, 1e-9));
  }

  @Test
  void runIntakeCommandsLeaderWithTorqueCurrent() {
    FakeTorqueCurrentMotorIO rollerLead = new FakeTorqueCurrentMotorIO();
    FakeTorqueCurrentMotorIO rollerFollow = new FakeTorqueCurrentMotorIO();
    FakeMotorIO arm = new FakeMotorIO();

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);

    intake.runIntake();

    assertAll(
        () -> assertNotNull(rollerLead.lastTorqueCurrent),
        () -> assertEquals(90.0, rollerLead.lastTorqueCurrent.in(Amps), 1e-9),
        () -> assertEquals(0.8, rollerLead.lastMaxAbsDutyCycle, 1e-9),
        () -> assertNull(rollerFollow.lastTorqueCurrent),
        () -> assertNull(rollerFollow.lastMaxAbsDutyCycle));
  }

  @Test
  void rollerVoltageCommandsLeaderOnly() {
    FakeMotorIO rollerLead = new FakeMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeMotorIO arm = new FakeMotorIO();

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);

    intake.runIntakeRollerVoltage(Volts.of(6.0));

    assertAll(
        () -> assertNotNull(rollerLead.lastVoltage),
        () -> assertEquals(6.0, rollerLead.lastVoltage.in(Volts), 1e-9),
        () -> assertNull(rollerFollow.lastVoltage));
  }

  @Test
  void rollerPercentageCommandsLeaderOnly() {
    FakeMotorIO rollerLead = new FakeMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeMotorIO arm = new FakeMotorIO();

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);

    intake.runIntakeRollerPercentage(0.25);

    assertAll(
        () -> assertNotNull(rollerLead.lastPercentage),
        () -> assertEquals(0.25, rollerLead.lastPercentage, 1e-9),
        () -> assertNull(rollerFollow.lastPercentage));
  }

  @Test
  void rollerRpmCommandsLeaderOnly() {
    FakeMotorIO rollerLead = new FakeMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeMotorIO arm = new FakeMotorIO();

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);

    intake.runIntakeRollerRPM(1_500.0);

    assertAll(
        () -> assertNotNull(rollerLead.lastRpm),
        () -> assertEquals(1_500.0, rollerLead.lastRpm, 1e-9),
        () -> assertNull(rollerFollow.lastRpm));
  }

  @Test
  void runOuttakeCommandsRollerTorqueCurrentWithDutyCycleCap() {
    FakeTorqueCurrentMotorIO rollerLead = new FakeTorqueCurrentMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeMotorIO arm = new FakeMotorIO();

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);

    intake.runOuttake();

    assertAll(
        () -> assertNotNull(rollerLead.lastTorqueCurrent),
        () -> assertEquals(-80.0, rollerLead.lastTorqueCurrent.in(Amps), 1e-9),
        () -> assertNotNull(rollerLead.lastMaxAbsDutyCycle),
        () -> assertEquals(0.8, rollerLead.lastMaxAbsDutyCycle, 1e-9));
  }

  @Test
  void runTorqueCurrentClampsDutyCycleCapBeforeCommandingTorqueCurrentMotor() {
    FakeTorqueCurrentMotorIO rollerLead = new FakeTorqueCurrentMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeMotorIO arm = new FakeMotorIO();

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);

    intake.runIntakeRollerTorqueCurrentFOC(Amps.of(90.0), 1.25);
    assertEquals(1.0, rollerLead.lastMaxAbsDutyCycle, 1e-9);

    intake.runIntakeRollerTorqueCurrentFOC(Amps.of(90.0), -0.25);
    assertEquals(0.0, rollerLead.lastMaxAbsDutyCycle, 1e-9);
  }

  @Test
  void runIntakeFallsBackToCappedVoltageWhenTorqueCurrentUnsupported() {
    FakeMotorIO rollerLead = new FakeMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeMotorIO arm = new FakeMotorIO();

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);

    intake.runIntake();

    assertAll(
        () -> assertNotNull(rollerLead.lastVoltage),
        () -> assertEquals(9.6, rollerLead.lastVoltage.in(Volts), 1e-9));
  }

  @Test
  void runOuttakeFallsBackToCappedVoltageWhenTorqueCurrentUnsupported() {
    FakeMotorIO rollerLead = new FakeMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeMotorIO arm = new FakeMotorIO();

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);

    intake.runOuttake();

    assertAll(
        () -> assertNotNull(rollerLead.lastVoltage),
        () -> assertEquals(-9.6, rollerLead.lastVoltage.in(Volts), 1e-9));
  }

  @Test
  void runTorqueCurrentFallbackClampsDutyCycleCapBeforeCommandingVoltage() {
    FakeMotorIO rollerLead = new FakeMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeMotorIO arm = new FakeMotorIO();

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);

    intake.runIntakeRollerTorqueCurrentFOC(Amps.of(90.0), 1.25);
    assertEquals(12.0, rollerLead.lastVoltage.in(Volts), 1e-9);

    intake.runIntakeRollerTorqueCurrentFOC(Amps.of(90.0), -0.25);
    assertEquals(0.0, rollerLead.lastVoltage.in(Volts), 1e-9);
  }

  @Test
  void zeroTorqueCurrentFallbackStopsLeader() {
    FakeMotorIO rollerLead = new FakeMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeMotorIO arm = new FakeMotorIO();

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);

    intake.runIntakeRollerTorqueCurrentFOC(Amps.of(0.0), 0.8);

    assertAll(
        () -> assertNotNull(rollerLead.lastVoltage),
        () -> assertEquals(0.0, rollerLead.lastVoltage.in(Volts), 1e-9),
        () -> assertNull(rollerFollow.lastVoltage));
  }

  @Test
  void torqueCurrentFallbackCommandsLeaderOnly() {
    FakeMotorIO rollerLead = new FakeMotorIO();
    FakeMotorIO rollerFollow = new FakeMotorIO();
    FakeMotorIO arm = new FakeMotorIO();

    IntakeSubsystem intake = new IntakeSubsystem(rollerLead, rollerFollow, arm);

    intake.runIntakeRollerTorqueCurrentFOC(Amps.of(90.0), 0.5);

    assertAll(
        () -> assertNotNull(rollerLead.lastVoltage),
        () -> assertEquals(6.0, rollerLead.lastVoltage.in(Volts), 1e-9),
        () -> assertNull(rollerFollow.lastVoltage));

    intake.runIntakeRollerTorqueCurrentFOC(Amps.of(-80.0), 0.5);

    assertAll(
        () -> assertEquals(-6.0, rollerLead.lastVoltage.in(Volts), 1e-9),
        () -> assertNull(rollerFollow.lastVoltage));
  }

  private static class FakeMotorIO implements MotorIO {
    protected double position;
    protected Double lastPositionSetpoint;
    protected Voltage lastVoltage;
    protected Double lastPercentage;
    protected Double lastRpm;

    @Override
    public void setMotorVoltage(Voltage voltage) {
      lastVoltage = voltage;
    }

    @Override
    public void setMotorPercentage(double percentage) {
      lastPercentage = percentage;
    }

    @Override
    public void setMotorRPM(double rpm) {
      lastRpm = rpm;
    }

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
    private Double lastMaxAbsDutyCycle;

    @Override
    public void setMotorTorqueCurrent(Current torqueCurrent) {
      lastTorqueCurrent = torqueCurrent;
    }

    @Override
    public void setMotorTorqueCurrent(Current torqueCurrent, double maxAbsDutyCycle) {
      lastTorqueCurrent = torqueCurrent;
      lastMaxAbsDutyCycle = maxAbsDutyCycle;
    }
  }
}
