package frc.robot.commands.shooter.hood;

import java.util.function.Supplier;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.PercentOutputValue;

import frc.robot.subsystems.shooter.HoodSubsystem;

public class HoodSetPercentOutputCommand extends SuppliedCommand<PercentOutputValue> {
  private HoodSubsystem m_hood;

  public HoodSetPercentOutputCommand(HoodSubsystem hood) {
    this(hood, 0.0);
  }

  public HoodSetPercentOutputCommand(HoodSubsystem hood, double percentOutput) {
    this(hood, () -> new PercentOutputValue(percentOutput));
  }

  public HoodSetPercentOutputCommand(HoodSubsystem hood, Supplier<PercentOutputValue> supplier) {
    super(supplier);
    addRequirements(hood);
    m_hood = hood;
  }

  @Override
  public void execute() {
    m_hood.setVoltage(12.0 * m_supplier.get().value);
  }
}
