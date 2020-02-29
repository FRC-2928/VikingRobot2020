package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.FeederSubsystem;

public class FeederStopCommand extends CommandBase {
  private FeederSubsystem m_feeder;

  public FeederStopCommand(FeederSubsystem feeder) {
    addRequirements(feeder);
    m_feeder = feeder;
  }

  @Override
  public void execute() {
    m_feeder.stop();
  }
}
