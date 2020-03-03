package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.commands.indexer.feeder.FeederStopCommand;
import frc.robot.commands.indexer.hopper.HopperStopCommand;
import frc.robot.subsystems.indexer.FeederSubsystem;
import frc.robot.subsystems.indexer.HopperSubsystem;

public class IndexerStopCommand extends ScheduleCommand {
  public IndexerStopCommand(FeederSubsystem feeder, HopperSubsystem hopper) {
    super(
      new FeederStopCommand(feeder),
      new HopperStopCommand(hopper)
    );
  }
}
