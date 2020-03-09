package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.oi.Bindable;
import frc.robot.oi.bindings.BartPrimaryOI;
import frc.robot.subsystems.Subsystems;

public class RobotContainer {
  private final Subsystems m_subsystems = new Subsystems();

  public RobotContainer() {
    configureButtonBindings();
  }

  public void configureButtonBindings() {
    for (var bindable : new Bindable[]{
      new BartPrimaryOI(new XboxController(0), m_subsystems),
    }) {
      bindable.bind();
    }
  }

  public Command getAutonomousCommand() {
    return new PrintCommand("auto not implemented");
  }
}
