package frc.robot.commands.indexer;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.indexer.FeederSubsystem;
import frc.robot.subsystems.indexer.HopperSubsystem;

public class IndexerFeedCommand extends RunCommand {
  public IndexerFeedCommand(FeederSubsystem feeder, HopperSubsystem hopper, BooleanSupplier feedSupplier) {
    super(() -> {
      if (feedSupplier.getAsBoolean()) {
        feeder.setPercentOutput(0.7);
        hopper.setPercentOutput(0.7);
      } else {
        feeder.stop();
        hopper.stop();
      }
    }, feeder, hopper);
  }
}
