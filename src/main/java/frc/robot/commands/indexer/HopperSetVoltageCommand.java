package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.HopperSubsystem;

public class HopperSetVoltageCommand extends CommandBase {
  private HopperSubsystem m_hopper;
  private DoubleSupplier m_voltageSupplier;

  public HopperSetVoltageCommand(HopperSubsystem hopper, double voltageVolts) {
    this(hopper, () -> voltageVolts);
  }

  public HopperSetVoltageCommand(HopperSubsystem hopper, DoubleSupplier voltageSupplier) {
    m_hopper = hopper;
    m_voltageSupplier = voltageSupplier;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_hopper.setVoltage(m_voltageSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
