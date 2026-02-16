package frc.robot.subsystems.climber;

import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class ClimberSubsystem extends SubsystemBase {

  public enum ClimberState {
    STOWED, 
    STOWING,
    DEPLOYING, 
    DEPLOYED
  }

  protected ClimberState currentClimberState;
  protected ClimberState desiredClimberState;

  protected ClimberIO climberMotorIO;
  protected ClimberIOInputs climberMotorInputs;

  public ClimberSubsystem(ClimberIO climbMotorIO) {
    this.climberMotorIO = climbMotorIO;
    this.climberMotorInputs = new ClimberIOInputs();

    // initailize climber states
    this.desiredClimberState = ClimberState.STOWED;
    this.currentClimberState = ClimberState.STOWED;
  }

  public void setDesiredState(ClimberState state) {
    this.desiredClimberState = state;

    switch (desiredClimberState) {
      case STOWED:
        desiredClimberState = ClimberState.STOWED;
        break;
      case STOWING:
        desiredClimberState = ClimberState.STOWING;

        break;
      case DEPLOYING: 
      default:

        desiredClimberState = ClimberState.DEPLOYING;
        break;

    }
  }


  /* Climber Actions */
  public void deployClimber() {
    climberMotorIO.setMotorPosition(CLIMBER_L1_SETPOINT);
  }

  public void stowClimber() {
    climberMotorIO.setMotorPosition(CLIMBER_HOME_SETPOINT);
  }

  public void stopClimber() {
    climberMotorIO.setMotorVoltage(0.0);
  }
  
  /* getters */
  public ClimberState getCurrentState() {
    return currentClimberState;
  }

  public ClimberState getDesiredState() {
    return desiredClimberState;
  }

  @Override
  public void periodic() {
    climberMotorIO.updateInputs(climberMotorInputs);
     // State machine logic
    switch (currentClimberState) {
      case STOWED:
        // Climber is stowed, do nothing
        // Check if we need to deploy
        if (desiredClimberState == ClimberState.DEPLOYED) {
          currentClimberState = ClimberState.DEPLOYING;
        }
        break;

      case STOWING:
        // Actively moving to stowed position
        stowClimber();
        currentClimberState = ClimberState.STOWED;
        break;

      case DEPLOYING:
        // Actively moving to deployed position
        deployClimber();
          currentClimberState = ClimberState.DEPLOYED;
        break;

      case DEPLOYED:
        // Climber is deployed, hold position
        // Check if we need to stow
        if (desiredClimberState == ClimberState.STOWED) {
          currentClimberState = ClimberState.STOWING;
        }
        break;
      default:
        break;
    }
    }
  }