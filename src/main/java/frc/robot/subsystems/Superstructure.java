package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Superstructure extends SubsystemBase {

  private final CommandSwerveDrivetrain swerve;
  private final IntakeSubsystem intake;
  private final HopperSubsystem hopper;
  private final ShooterSubsystem shooter;
  private final ClimberSubsystem climber;
  private final VisionSubsystem vision;
  private final RobotContainer container;

  

  public Superstructure(
      CommandSwerveDrivetrain swerve,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      ShooterSubsystem shooter,
      ClimberSubsystem climber,
      VisionSubsystem vision,
      RobotContainer container) {
        this.swerve = swerve;
        this.intake = intake;
        this.hopper = hopper;
        this.shooter = shooter;
        this.climber = climber;
        this.vision = vision;
        this.container = container;
  }
}
