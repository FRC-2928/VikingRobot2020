package frc.robot.commands.indexer.feeder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.FeederSubsystem;

public class FeederSetPercentOutputCommand extends CommandBase {
  private FeederSubsystem m_feeder;
  private DoubleSupplier m_supplier;

  public FeederSetPercentOutputCommand(FeederSubsystem feeder, double value) {
    this(feeder, () -> value);
  }

  public FeederSetPercentOutputCommand(FeederSubsystem feeder, DoubleSupplier supplier) {
    addRequirements(feeder);
    m_feeder = feeder;
  }

  @Override
  public void execute() {
    m_feeder.setPercentOutput(m_supplier.getAsDouble());
  }
}
