package frc.robot.subsystems.climber;

import static frc.robot.Constants.ClimberConstants.*;

import dev.doglog.DogLog;
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

    // Set current state to transition state based on desired state
    switch (desiredClimberState) {
      case STOWED:
        currentClimberState = ClimberState.STOWING;
        break;
      case DEPLOYED:
        currentClimberState = ClimberState.DEPLOYING;
        break;
      default:
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
  public void handleStateTransition() {
    // State machine logic
      switch (currentClimberState) {
        case STOWED:
          // Continuosly stow the climber
          stowClimber();
          break;

        case STOWING:
          // Actively moving to stowed position
          // check if the arm is stowed/deployed before switching
          stowClimber();
          if (Math.abs(climberMotorInputs.getMotorPosition() - CLIMBER_HOME_ANGLE) < 1.0) {
            currentClimberState = ClimberState.STOWED;
          } 
          break;

        case DEPLOYING:
          // Actively moving to deployed position
          deployClimber();
          // check if we are in deployed position
          if (Math.abs(climberMotorInputs.getMotorPosition() - CLIMBER_L1_SETPOINT) < 1.0) {
            currentClimberState = ClimberState.DEPLOYED;
          }
          break;

        case DEPLOYED:
          // Climber is deployed, hold position
          deployClimber();
          break;
        default:
          break;
    }
  }


  @Override
  public void periodic() {
      handleStateTransition();
      DogLog.log("CLimber/Climber State", currentClimberState.toString());

      climberMotorIO.updateInputs(climberMotorInputs);

    }
  }