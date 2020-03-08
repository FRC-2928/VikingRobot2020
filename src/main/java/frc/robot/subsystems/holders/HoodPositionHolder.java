package frc.robot.subsystems.holders;

import org.ballardrobotics.subsystems.Holder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodPositionHolder extends Holder<Double> {
  public HoodPositionHolder() {
    super(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("hood_holder", m_value);
  }
}
