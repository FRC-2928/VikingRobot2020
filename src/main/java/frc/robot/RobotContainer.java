package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.oi.OIBindable;
import frc.robot.oi.OIBinder;
import frc.robot.oi.impl.BartPrimaryOI;
import frc.robot.subsystems.Subsystems;

public class RobotContainer {
  private final Subsystems m_subsystems;

  public RobotContainer() {
    m_subsystems = new Subsystems();

    var operatorInterfaces = new OIBindable[]{
      new BartPrimaryOI(new XboxController(0)),
    };
    for (var oi : operatorInterfaces) {
      OIBinder.bind(m_subsystems, oi);
    }
  }

  public Command getAutonomousCommand() {
    return new PrintCommand("auto not implemented");
  }
}
