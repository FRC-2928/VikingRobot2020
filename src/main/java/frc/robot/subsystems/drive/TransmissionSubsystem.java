package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.TransmissionConstants;
import frc.robot.commands.drive.TransmissionSetHighGearCommand;

public class TransmissionSubsystem extends SubsystemBase {
  private Solenoid m_solenoid;
  private boolean m_isHighGear;

  public static TransmissionSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createFake();
  }

  public static TransmissionSubsystem createReal() {
    var solenoid = new Solenoid(TransmissionConstants.kSolenoidChannel);
    return new TransmissionSubsystem(solenoid);
  }

  public static TransmissionSubsystem createFake() {
    var solenoid = new Solenoid(TransmissionConstants.kSolenoidChannel);
    return new TransmissionSubsystem(solenoid);
  }

  public TransmissionSubsystem(Solenoid solenoid) {
    m_solenoid = solenoid;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("drive_is_high_gear", m_isHighGear);
  }

  public boolean isHighGear() {
    return m_isHighGear;
  }

  public void setLowGear() {
    m_isHighGear = false;
    m_solenoid.set(false);
  }

  public void setHighGear() {
    m_isHighGear = true;
    m_solenoid.set(true);
  }
}
