package frc.robot.subsystems.holders;

import org.ballardrobotics.subsystems.Holder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FlywheelVelocityHolder extends Holder<Double> {
  public FlywheelVelocityHolder() {
    super(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("flywheel_holder", m_value);
  }
}
