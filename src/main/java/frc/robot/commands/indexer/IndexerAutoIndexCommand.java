package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.FeederSubsystem;
import frc.robot.subsystems.indexer.HopperSubsystem;

// TODO: Implementation
public class IndexerAutoIndexCommand extends CommandBase {
  private FeederSubsystem m_feeder;
  private HopperSubsystem m_hopper;

  public IndexerAutoIndexCommand(FeederSubsystem feeder, HopperSubsystem hopper) {
    addRequirements(feeder, hopper);

    m_feeder = feeder;
    m_hopper = hopper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
