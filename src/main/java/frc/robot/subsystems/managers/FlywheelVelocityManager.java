package frc.robot.subsystems.managers;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.managers.FlywheelManagerSetTargetVelocityCommand;

public class FlywheelVelocityManager extends SubsystemBase {
  private double m_rpm;

  public FlywheelVelocityManager() {
    m_rpm = 0.0;
  }

  public void configureShuffleboard(ShuffleboardLayout stateLayout, ShuffleboardLayout controlLayout) {
    stateLayout.addNumber("rpm", this::getTargetRPM);

    var entry = controlLayout.add("target_rpm", 0.0).getEntry();
    controlLayout.add("use_target", new FlywheelManagerSetTargetVelocityCommand(this, () -> entry.getDouble(0.0)));
  }

  public void setTargetRPM(double rpm) {
    m_rpm = rpm;
  }

  public double getTargetRPM() {
    return m_rpm;
  }
}
