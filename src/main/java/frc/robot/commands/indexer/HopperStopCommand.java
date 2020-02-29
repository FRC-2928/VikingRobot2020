package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.HopperSubsystem;

public class HopperStopCommand extends CommandBase {
  private HopperSubsystem m_hopper;

  public HopperStopCommand(HopperSubsystem hopper) {
    addRequirements(hopper);
    m_hopper = hopper;
  }

  @Override
  public void execute() {
    m_hopper.stop();
  }
}
