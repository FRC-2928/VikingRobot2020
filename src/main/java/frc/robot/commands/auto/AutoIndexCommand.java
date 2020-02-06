package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.FeederSubsystem;
import frc.robot.subsystems.indexer.HopperSubsystem;

/**
 * AutoIndexCommand automatically manages the Power Cells using the state
 * of 3 break beam sensors to determine how to drive the system.
 */
public class AutoIndexCommand extends CommandBase {
  private HopperSubsystem m_hopper;
  private FeederSubsystem m_feeder;

  public AutoIndexCommand(HopperSubsystem hopper, FeederSubsystem feeder) {
    addRequirements(hopper, feeder);

    m_hopper = hopper;
    m_feeder = feeder;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_hopper.stop();
    m_feeder.stop();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
