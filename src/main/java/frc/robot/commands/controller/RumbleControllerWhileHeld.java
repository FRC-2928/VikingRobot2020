package frc.robot.commands.controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RumbleControllerWhileHeld extends CommandBase {
  private double m_rumble;
  private XboxController m_controller;

  public RumbleControllerWhileHeld(XboxController controller, double rumble) {
    m_controller = controller;
    m_rumble = rumble;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_controller.setRumble(RumbleType.kLeftRumble, m_rumble);
    m_controller.setRumble(RumbleType.kRightRumble, m_rumble);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controller.setRumble(RumbleType.kLeftRumble, 0);
    m_controller.setRumble(RumbleType.kRightRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
