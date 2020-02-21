package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

/**
 * TransmissionSubsystem is responsible for shifting the gear on the drivetrain
 * Contains a statemachine for keeping gear state
 */

public class TransmissionSubsystem extends SubsystemBase {
  private Solenoid m_shiftPiston;
  private GearState m_gearState;

  public enum GearState {
    HIGH, LOW;
  }

  public TransmissionSubsystem() {

    m_shiftPiston = new Solenoid(RobotMap.kDrivetrainShiftSolenoid);

    m_gearState = GearState.HIGH;

  }

  public void setGearState(GearState state) {
    m_gearState = state;

    switch (state) {

    case HIGH:
      setFalse();
      break;

    case LOW:
      setTrue();
      break;
    }
  }

  public void setHigh(){
    setGearState(GearState.HIGH);
  }

  public void setLow(){
    setGearState(GearState.LOW);
  }

  public void toggle() {
    setGearState(m_gearState == GearState.LOW ? GearState.HIGH : GearState.LOW);
  }

  public GearState getGearState() {
    return m_gearState;
  }

  private void setTrue() {
    m_shiftPiston.set(true);
  }

  private void setFalse() {
    m_shiftPiston.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
