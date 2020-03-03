package frc.robot.commands.indexer.hopper;

import java.util.function.Supplier;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.PercentOutputValue;

import frc.robot.subsystems.indexer.HopperSubsystem;

public class HopperSetPercentOutputCommand extends SuppliedCommand<PercentOutputValue> {
  private HopperSubsystem m_hopper;

  public HopperSetPercentOutputCommand(HopperSubsystem hopper) {
    this(hopper, 0.0);
  }

  public HopperSetPercentOutputCommand(HopperSubsystem hopper, double percentOutput) {
    this(hopper, () -> new PercentOutputValue(percentOutput));
  }

  public HopperSetPercentOutputCommand(HopperSubsystem hopper, Supplier<PercentOutputValue> supplier) {
    super(supplier);
    addRequirements(hopper);
    m_hopper = hopper;
  }

  @Override
  public void execute() {
    m_hopper.setPercentOutput(m_supplier.get().value);
  }
}
