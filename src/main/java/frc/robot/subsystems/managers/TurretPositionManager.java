package frc.robot.subsystems.managers;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.managers.TurretManagerSetTargetPositionCommand;

public class TurretPositionManager extends SubsystemBase {
  private double m_degrees;

  public TurretPositionManager() {
    m_degrees = 0.0;
  }

  public void configureShuffleboard(ShuffleboardLayout stateLayout, ShuffleboardLayout controlLayout) {
    stateLayout.addNumber("degrees", this::getTargetDegrees);

    var entry = controlLayout.add("target_degrees", 0.0).getEntry();
    controlLayout.add("use_target", new TurretManagerSetTargetPositionCommand(this, () -> entry.getDouble(0.0)));
  }

  public void setTargetDegrees(double rpm) {
    m_degrees = rpm;
  }

  public double getTargetDegrees() {
    return m_degrees;   
  }
}
