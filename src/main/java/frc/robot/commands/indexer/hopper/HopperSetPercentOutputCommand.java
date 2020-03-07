package frc.robot.commands.indexer.hopper;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.HopperSubsystem;

public class HopperSetPercentOutputCommand extends CommandBase {
  private HopperSubsystem m_hopper;
  private DoubleSupplier m_supplier;

  public HopperSetPercentOutputCommand(HopperSubsystem hopper, double percentOutput) {
    this(hopper, () -> percentOutput);
  }

  public HopperSetPercentOutputCommand(HopperSubsystem hopper, DoubleSupplier supplier) {
    addRequirements(hopper);
    m_hopper = hopper;
    m_supplier = supplier;
  }

  @Override
  public void execute() {
    m_hopper.setPercentOutput(m_supplier.getAsDouble());
  }
}
