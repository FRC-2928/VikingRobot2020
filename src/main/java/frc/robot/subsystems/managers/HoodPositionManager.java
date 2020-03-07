package frc.robot.subsystems.managers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodPositionManager extends SubsystemBase {
  private double m_degrees;

  public HoodPositionManager() {
    m_degrees = 0.0;
  }

  public void setTargetDegrees(double rpm) {
    m_degrees = rpm;
  }

  public double getTargetDegrees() {
    return m_degrees;
  }
}
