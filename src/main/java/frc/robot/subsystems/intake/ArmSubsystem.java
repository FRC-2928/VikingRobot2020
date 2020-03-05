package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

public class ArmSubsystem extends SubsystemBase {
  private Solenoid m_solenoidArm;
  private Solenoid m_solenoidBase;
  
  public enum IntakeState {
    GROUND_PICKUP, OPEN, STOWED; //Change OPEN later
  }

  private IntakeState currentState;

  public ArmSubsystem() {
    m_solenoidArm = new Solenoid(RobotMap.kIntakeArmSolenoid);
    m_solenoidBase = new Solenoid(RobotMap.kIntakeBaseSolenoid);
    
    currentState = IntakeState.STOWED;
  }

  public void groundPickup() {
    moveIntake(IntakeState.GROUND_PICKUP);
  }

  public void openIntake() {
    moveIntake(IntakeState.OPEN);
  }

  // This is the default command
  public void stowIntake() {
    moveIntake(IntakeState.STOWED);
  }

  public void moveIntake(IntakeState state) {

    switch (state) {
      case GROUND_PICKUP: 
        m_solenoidArm.set(true);
        m_solenoidBase.set(true);
      break;

      case OPEN: 
        m_solenoidArm.set(true);
        m_solenoidBase.set(false);
      break;

      case STOWED:
        m_solenoidArm.set(false);
        m_solenoidBase.set(false);
      break;

      default:
      break;
      }

      currentState = state;
    }

    public IntakeState getIntakeState(){
      return currentState;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
