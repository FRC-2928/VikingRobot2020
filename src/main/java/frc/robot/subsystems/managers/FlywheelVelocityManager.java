package frc.robot.subsystems.managers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelVelocityManager extends SubsystemBase {
  private double m_rpm;

  public FlywheelVelocityManager() {
    m_rpm = 0.0;
  }

  public void setTargetRPM(double rpm) {
    m_rpm = rpm;
  }

  public double getTargetRPM() {
    return m_rpm;
  }
}
