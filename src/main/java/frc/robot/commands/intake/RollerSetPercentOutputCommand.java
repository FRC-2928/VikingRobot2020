package frc.robot.commands.intake;

import java.util.function.Supplier;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.PercentOutputValue;

import frc.robot.subsystems.intake.RollerSubsystem;

public class RollerSetPercentOutputCommand extends SuppliedCommand<PercentOutputValue> {
  private RollerSubsystem m_roller;

  public RollerSetPercentOutputCommand(RollerSubsystem roller) {
    this(roller, 0.0);
  }

  public RollerSetPercentOutputCommand(RollerSubsystem roller, double value) {
    this(roller, () -> new PercentOutputValue(value));
  }

  public RollerSetPercentOutputCommand(RollerSubsystem roller, Supplier<PercentOutputValue> supplier) {
    super(supplier);
    addRequirements(roller);
    m_roller = roller;
  }

  @Override
  public void execute() {
    m_roller.setPercentOutput(m_supplier.get().value);
  }
}
