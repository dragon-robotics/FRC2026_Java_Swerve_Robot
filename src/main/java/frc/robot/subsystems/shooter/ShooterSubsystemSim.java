package frc.robot.subsystems.shooter;

import static frc.robot.Constants.IntakeSubsystemConstants.INTAKE_ARM_GEAR_RATIO;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.intake.IntakeIOSparkMaxSim;
import frc.robot.subsystems.intake.IntakeIOTalonFXSim;

public class ShooterSubsystemSim extends ShooterSubsystem {

  // Simulation Display
  Mechanism2d mech;

  // Visualization constants
  private final double HEIGHT = 1; // Controls the height of the mech2d SmartDashboard
  private final double WIDTH = 1; // Controls the height of the mech2d SmartDashboard

  // Velocity Indicator
  private final MechanismLigament2d velocityMech, midline;

  // Position
  private final MechanismLigament2d arm, side1, side2, side3, side4, side5, side6, side7, side8;

  private final FlywheelSim flywheelSim;

  public ShooterSubsystemSim(
      ShooterIO shooterLeadIO,
      ShooterIO shooterFollowIO,
      ShooterIO shooterKickerIO,
      ShooterIO shooterHoodIO) {
    super(shooterLeadIO, shooterFollowIO, shooterKickerIO, shooterHoodIO);

    mech = new Mechanism2d(WIDTH, HEIGHT);

    velocityMech =
        mech
            .getRoot("velocityLineReferencePosition", 0.75, 0.5)
            .append(
                new MechanismLigament2d(
                    "velocityLine",
                    1,
                    90,
                    6,
                    new Color8Bit(Color.kAliceBlue)));
    midline = mech.getRoot("midline", 0.7, 0.5)
      .append(new MechanismLigament2d("midline", 0.1, 0, 3, new Color8Bit(Color.kCyan)));
    arm = mech.getRoot("pivotPoint", 0.25, 0.5)
      .append(new MechanismLigament2d("arm", .2, 0, 0, new Color8Bit(Color.kAliceBlue)));
    side1 = arm
      .append(new MechanismLigament2d("side1", 0.15307, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
    side2 = side1
      .append(new MechanismLigament2d("side2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    side3 = side2
      .append(new MechanismLigament2d("side3", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    side4 = side3
      .append(new MechanismLigament2d("side4", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    side5 = side4
      .append(new MechanismLigament2d("side5", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    side6 = side5
      .append(new MechanismLigament2d("side6", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    side7 = side6
      .append(new MechanismLigament2d("side7", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
    side8 = side7
      .append(new MechanismLigament2d("side8", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));    

    DCMotor shooterMotorType =
        shooterLeadIO instanceof ShooterIOTalonFXSim
            ? DCMotor.getKrakenX60Foc(2)
            : DCMotor.getNEO(2);

    flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            shooterMotorType,
            0.007,
            0.5),
        shooterMotorType, 0.1);

    SmartDashboard.putData("Shooter Flywheel Sim", mech);
  }

  /** Update simulation. */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    // Note: This may need to be talonfx.getSimState().getMotorVoltage() as the input.
    // armSim.setInput(dcMotor.getVoltage(dcMotor.getTorque(armSim.getCurrentDrawAmps()),
    // armSim.getVelocityRadPerSec()));
    // armSim.setInput(getVoltage());
    // Sets input voltage based on whether it is talon fx or not
    // Use motor voltage for TalonFX simulation input, otherwise get the motor voltage from inputs
    if (shooterLeadIO instanceof ShooterIOTalonFXSim talonIO) {
      flywheelSim.setInput(talonIO.getSimState().getMotorVoltage());
    } else {
      flywheelSim.setInput(shooterLeadInputs.getMotorVoltage());
    }

    // Update simulation by 20ms
    flywheelSim.update(0.020);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));

    double motorPosition =
        Radians.of(flywheelSim.getAngularVelocityRPM() * INTAKE_ARM_GEAR_RATIO).in(Rotations);
    double motorVelocity =
        RadiansPerSecond.of(flywheelSim.getVelocityRadPerSec() * INTAKE_ARM_GEAR_RATIO)
            .in(RotationsPerSecond);

    if (shooterLeadIO instanceof ShooterIOTalonFXSim talonIO) {
      talonIO.getSimState().setRawRotorPosition(motorPosition);
      talonIO.getSimState().setRotorVelocity(motorVelocity);
    } else {
      ((ShooterIOSparkMaxSim) shooterLeadIO)
          .getMotorSim()
          .iterate(motorVelocity, RoboRioSim.getVInVoltage(), 0.02);
    }
  }

  @Override
  public void periodic() {
    // Update arm angle
    double currentAngleRad = flywheelSim.getAngleRads();
    armMech.setAngle(Units.radiansToDegrees(currentAngleRad));

    handleStateTransition();

    // Add telemetry data
    DogLog.log("Intake/Intake State", currIntakeState.toString());
    DogLog.log(
        "Intake/Intake Arm Angle (deg)", String.valueOf(Units.radiansToDegrees(currentAngleRad)));
    DogLog.log(
        "Intake/Intake Arm Velocity (deg/s)",
        String.valueOf(Units.radiansToDegrees(intakeArmSim.getVelocityRadPerSec())));
    DogLog.log("Intake/Intake Arm Current (A)", String.valueOf(intakeArmSim.getCurrentDrawAmps()));

    // This method will be called once per scheduler run
    intakeRollerIO.updateInputs(intakeRollerInputs);
    shooterLeadIO.updateInputs(intakeArmInputs);
  }  

}
