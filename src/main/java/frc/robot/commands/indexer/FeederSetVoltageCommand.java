package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.FeederSubsystem;

public class FeederSetVoltageCommand extends CommandBase {
  private FeederSubsystem m_feeder;
  private DoubleSupplier m_voltageSupplier;

  public FeederSetVoltageCommand(FeederSubsystem feeder, double voltageVolts) {
    this(feeder, () -> voltageVolts);
  }

  public FeederSetVoltageCommand(FeederSubsystem feeder, DoubleSupplier voltageSupplier) {
    m_feeder = feeder;
    m_voltageSupplier = voltageSupplier;
  }

  @Override
  public void execute() {
    m_feeder.setVoltage(m_voltageSupplier.getAsDouble());
  }
}
