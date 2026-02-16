package frc.robot.subsystems.climber;

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
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.robot.subsystems.intake.IntakeIOSparkMaxSim;
import frc.robot.subsystems.intake.IntakeIOTalonFXSim;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.IntakeSubsystemConstants.INTAKE_ARM_GEAR_RATIO;

import dev.doglog.DogLog;

public class ClimberSubsystemSim extends ClimberSubsystem{
    private final Mechanism2d mech; 
    private final MechanismRoot2d root; 
    private final MechanismLigament2d climberMech;

    // visual constants 
    private final double BASE_WIDTH = 40.0;
    private final double BASE_HEIGHT = 20.0;
    private final double TOWER_HEIGHT = 30.0;
    private final double ARM_WIDTH = 10.0;

    private final double climberLength; 
    private final double visualScaleFactor; 

    private final SingleJointedArmSim climberSim; 

    public ClimberSubsystemSim(ClimberIO climberMotorIO) {
        super(climberMotorIO);
        
        climberLength = 1; 

        visualScaleFactor = 200.0 / climberLength; // Scale to ~200 pixels

        // Create the simulation display
        mech = new Mechanism2d(400, 400);
        root = mech.getRoot("ArmRoot", 200, 200);

        MechanismLigament2d armBase =
            root.append(
                new MechanismLigament2d(
                    "Base", BASE_WIDTH, 0, BASE_HEIGHT, new Color8Bit(Color.kDarkGray)));

        climberMech =
            root.append(
                new MechanismLigament2d(
                    "Climber", climberLength * visualScaleFactor, 90, ARM_WIDTH, new Color8Bit(Color.kBlue)));
        

        var motorType =
            climberMotorIO instanceof IntakeIOTalonFXSim
                ? ((IntakeIOTalonFXSim) climberMotorIO).getMotorType()
                : ((IntakeIOSparkMaxSim) climberMotorIO).getMotorType();
        
        climberSim =
            new SingleJointedArmSim(
                motorType,
                CLIMBER_GEAR_RATIO,
                SingleJointedArmSim.estimateMOI(
                    CLIMBER_LENGTH_METERS,
                    CLIMBER_MASS_KG),
                CLIMBER_LENGTH_METERS,
                CLIMBER_MIN_ANGLE_RADIANS,
                CLIMBER_MAX_ANGLE_RADIANS,
                true,
                CLIMBER_STARTING_ANGLE_RADIANS
            );

        SmartDashboard.putData("Climber Sim", mech);
    }

    @Override
    public void simulationPeriodic() {

        // Apply motor voltage to physics sim
        if (climberMotorIO instanceof ClimberIOTalonFXSim talonIO) {
            climberSim.setInput(talonIO.getSimState().getMotorVoltage()); 
        } else {
            climberSim.setInput(climberMotorInputs.getMotorVelocity()); 
        }

        climberSim.update(0.020);
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(climberSim.getCurrentDrawAmps()));

        double motorPosition =
            Radians.of(climberSim.getAngleRads() * CLIMBER_GEAR_RATIO).in(Rotations);
        double motorVelocity =
            RadiansPerSecond.of(climberSim.getVelocityRadPerSec() * CLIMBER_GEAR_RATIO)
                .in(RotationsPerSecond);
                
        if (climberMotorIO instanceof IntakeIOTalonFXSim talonIO) {
            talonIO.getSimState().setRawRotorPosition(motorPosition);
            talonIO.getSimState().setRotorVelocity(motorVelocity);
        } else {
            ((IntakeIOSparkMaxSim) climberMotorIO)
                .getMotorSim()
                .iterate(motorVelocity, RoboRioSim.getVInVoltage(), 0.02);
        }
    }

    @Override
    public void periodic() {

        double currentAngleRad = climberSim.getAngleRads();
        climberMech.setAngle(Units.radiansToDegrees(currentAngleRad));
        
        handleStateTransition();

        DogLog.log("Climber/State", currentClimberState.toString());

        DogLog.log(
            "Climber/Hook Angle (deg)",
            String.valueOf(Units.radiansToDegrees(currentAngleRad)));

        DogLog.log(
            "Climber/Hook Velocity (deg/s)",
            String.valueOf(Units.radiansToDegrees(climberSim.getVelocityRadPerSec())));

        DogLog.log(
            "Climber/Hook Current (A)",
            String.valueOf(climberSim.getCurrentDrawAmps()));        

        climberMotorIO.updateInputs(climberMotorInputs);
    }

}
