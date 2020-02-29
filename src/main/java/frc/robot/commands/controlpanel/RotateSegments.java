package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Constants.ConversionConstants;
import frc.robot.subsystems.controlpanel.ControlPanelSubsystem;

/*
 * Rotates the control panel the specified number of segments
 */

public class RotateSegments extends CommandBase {

    // The subsystem the command runs on
    private final ControlPanelSubsystem m_controlPanel;
    private double m_reference;

    public RotateSegments(ControlPanelSubsystem subsystem, double reference) {
    m_controlPanel = subsystem;
    m_reference = reference;
    addRequirements(m_controlPanel);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_controlPanel.rotateSegments(m_reference);
  }

  @Override
  public boolean isFinished() {
    return m_controlPanel.atReference();
  }

}