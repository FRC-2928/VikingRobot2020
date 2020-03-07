package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.RollerSubsystem;

public class RollerSetPercentOutputCommand extends CommandBase {
  private RollerSubsystem m_roller;
  private DoubleSupplier m_supplier;

  public RollerSetPercentOutputCommand(RollerSubsystem roller, double value) {
    this(roller, () -> value);
  }

  public RollerSetPercentOutputCommand(RollerSubsystem roller, DoubleSupplier supplier) {
    addRequirements(roller);
    m_roller = roller;
    m_supplier = supplier;
  }

  @Override
  public void execute() {
    m_roller.setPercentOutput(m_supplier.getAsDouble());
  }
}
