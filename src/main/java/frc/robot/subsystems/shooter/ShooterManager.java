package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterManager extends SubsystemBase {
  private double m_hoodReference;
  private double m_flywheelReference;

  public ShooterManager() {
    m_hoodReference = 0;
    m_flywheelReference = 0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Manager Hood", m_hoodReference);
    SmartDashboard.putNumber("Shooter Manager Flywheel", m_flywheelReference);
  }

  public void setHoodReferernce(double reference){
    m_hoodReference = reference;
  }

  public double getHoodReference(){
    return m_hoodReference;
  }

  public void setFlywheelReference(double reference){
    m_flywheelReference = reference;
  }

  public double getFlywheelReference(){
    return m_flywheelReference;
  }
}
