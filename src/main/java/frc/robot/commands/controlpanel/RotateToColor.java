
package frc.robot.commands.controlpanel;



import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.controlpanel.ControlPanelSubsystem;


/**

 * Rotates the control panel the specified number of segments

 */

public class RotateToColor extends CommandBase {

  // The subsystem the command runs on

  private final ControlPanelSubsystem m_controlPanel;

  public RotateToColor(ControlPanelSubsystem subsystem) {
    m_controlPanel = subsystem;
    addRequirements(m_controlPanel);
  }

  @Override
  public void initialize() {
    m_controlPanel.rotateToColor();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

}