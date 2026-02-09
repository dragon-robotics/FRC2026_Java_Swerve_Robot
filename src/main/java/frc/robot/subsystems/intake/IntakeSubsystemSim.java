package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.IntakeSubsystemConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakeSubsystemSim extends IntakeSubsystem {

  // Simulation display
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d armMech;

  // Visualization constants
  private final double BASE_WIDTH = 40.0;
  private final double BASE_HEIGHT = 20.0;
  private final double TOWER_HEIGHT = 30.0;
  private final double ARM_WIDTH = 10.0;

  // Arm parameters
  private final double armLength;
  private final double visualScaleFactor;

  // Simulation
  private final SingleJointedArmSim intakeArmSim;

  /**
   * Creates a new visualization for the arm.
   *
   * @param armSubsystem The arm subsystem to visualize
   */
  public IntakeSubsystemSim(IntakeIO intakeRollerIO, IntakeIO intakeArmIO) {
    super(intakeRollerIO, intakeArmIO);

    // Get arm length from subsystem (in meters)
    armLength = 1;

    // Calculate scale factor to keep visualization in reasonable bounds
    visualScaleFactor = 200.0 / armLength; // Scale to ~200 pixels

    // Create the simulation display
    mech = new Mechanism2d(400, 400);
    root = mech.getRoot("ArmRoot", 200, 200);

    // Add arm base
    MechanismLigament2d armBase =
        root.append(
            new MechanismLigament2d(
                "Base", BASE_WIDTH, 0, BASE_HEIGHT, new Color8Bit(Color.kDarkGray)));

    // Add tower
    MechanismLigament2d tower =
        armBase.append(
            new MechanismLigament2d(
                "Tower", TOWER_HEIGHT, 90, BASE_HEIGHT / 2, new Color8Bit(Color.kGray)));

    // Add the arm pivot point
    MechanismLigament2d pivot =
        tower.append(new MechanismLigament2d("Pivot", 5, 0, 5, new Color8Bit(Color.kBlack)));

    // Add the arm
    armMech =
        pivot.append(
            new MechanismLigament2d(
                "Arm", armLength * visualScaleFactor, 0, ARM_WIDTH, new Color8Bit(Color.kBlue)));

    var motorType =
        intakeArmIO instanceof IntakeIOTalonFXSim
            ? ((IntakeIOTalonFXSim) intakeArmIO).getMotorType()
            : ((IntakeIOSparkMaxSim) intakeArmIO).getMotorType();

    // Initialize simulation
    intakeArmSim =
        new SingleJointedArmSim(
            motorType, // Motor type
            INTAKE_ARM_GEAR_RATIO, // Gear ratio
            SingleJointedArmSim.estimateMOI(
                INTAKE_ARM_LENGTH_METERS, INTAKE_ARM_MASS_KG), // Arm moment of inertia
            INTAKE_ARM_LENGTH_METERS, // Arm length (m)
            INTAKE_MIN_ANGLE_RADIANS, // Min angle (rad)
            INTAKE_MAX_ANGLE_RADIANS, // Max angle (rad)
            true, // Simulate gravity
            INTAKE_STARTING_ANGLE_RADIANS // Starting position (rad)
            );

    // Initialize visualization
    SmartDashboard.putData("Intake Arm Sim", mech);
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
    if (intakeArmIO instanceof IntakeIOTalonFXSim talonIO) {
      intakeArmSim.setInput(talonIO.getSimState().getMotorVoltage());
    } else {
      intakeArmSim.setInput(intakeArmInputs.getMotorVoltage());
    }

    // Update simulation by 20ms
    intakeArmSim.update(0.020);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(intakeArmSim.getCurrentDrawAmps()));

    double motorPosition =
        Radians.of(intakeArmSim.getAngleRads() * INTAKE_ARM_GEAR_RATIO).in(Rotations);
    double motorVelocity =
        RadiansPerSecond.of(intakeArmSim.getVelocityRadPerSec() * INTAKE_ARM_GEAR_RATIO)
            .in(RotationsPerSecond);

    if (intakeArmIO instanceof IntakeIOTalonFXSim talonIO) {
      talonIO.getSimState().setRawRotorPosition(motorPosition);
      talonIO.getSimState().setRotorVelocity(motorVelocity);
    } else {
      ((IntakeIOSparkMaxSim) intakeArmIO)
          .getMotorSim()
          .iterate(motorVelocity, RoboRioSim.getVInVoltage(), 0.02);
    }
  }

  @Override
  public void periodic() {
    // Update arm angle
    double currentAngleRad = intakeArmSim.getAngleRads();
    armMech.setAngle(Units.radiansToDegrees(currentAngleRad));

    // Add telemetry data
    SmartDashboard.putNumber("Intake Arm Angle (deg)", Units.radiansToDegrees(currentAngleRad));
    SmartDashboard.putNumber(
        "Intake Arm Velocity (deg/s)", Units.radiansToDegrees(intakeArmSim.getVelocityRadPerSec()));
    SmartDashboard.putNumber("Intake Arm Current (A)", intakeArmSim.getCurrentDrawAmps());
  }
}
