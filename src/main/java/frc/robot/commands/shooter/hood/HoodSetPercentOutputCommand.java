package frc.robot.commands.shooter.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class HoodSetPercentOutputCommand extends CommandBase {
  private HoodSubsystem m_hood;
  private DoubleSupplier m_supplier;

  public HoodSetPercentOutputCommand(HoodSubsystem hood) {
    this(hood, 0.0);
  }

  public HoodSetPercentOutputCommand(HoodSubsystem hood, double percentOutput) {
    this(hood, () -> percentOutput);
  }

  public HoodSetPercentOutputCommand(HoodSubsystem hood, DoubleSupplier supplier) {
    addRequirements(hood);
    m_hood = hood;
    m_supplier = supplier;
  }

  @Override
  public void execute() {
    m_hood.setVoltage(12.0 * m_supplier.getAsDouble());
  }
}
