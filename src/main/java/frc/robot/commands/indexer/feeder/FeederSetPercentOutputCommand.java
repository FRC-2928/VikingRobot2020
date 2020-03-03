package frc.robot.commands.indexer.feeder;

import java.util.function.Supplier;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.PercentOutputValue;

import frc.robot.subsystems.indexer.FeederSubsystem;

public class FeederSetPercentOutputCommand extends SuppliedCommand<PercentOutputValue> {
  private FeederSubsystem m_feeder;

  public FeederSetPercentOutputCommand(FeederSubsystem feeder) {
    this(feeder, 0.0);
  }

  public FeederSetPercentOutputCommand(FeederSubsystem feeder, double percentOutput) {
    this(feeder, () -> new PercentOutputValue(percentOutput));
  }

  public FeederSetPercentOutputCommand(FeederSubsystem feeder, Supplier<PercentOutputValue> supplier) {
    super(supplier);
    addRequirements(feeder);
    m_feeder = feeder;
  }

  @Override
  public void execute() {
    m_feeder.setPercentOutput(m_supplier.get().value);
  }
}
