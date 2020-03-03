package frc.robot.commands.shooter.turret;

import java.util.function.Supplier;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.PercentOutputValue;

import frc.robot.subsystems.shooter.TurretSubsystem;

public class TurretSetPercentOutputCommand extends SuppliedCommand<PercentOutputValue> {
  private TurretSubsystem m_turret;

  public TurretSetPercentOutputCommand(TurretSubsystem turret) {
    this(turret, 0.0);
  }

  public TurretSetPercentOutputCommand(TurretSubsystem turret, double percentOutput) {
    this(turret, () -> new PercentOutputValue(percentOutput));
  }

  public TurretSetPercentOutputCommand(TurretSubsystem turret, Supplier<PercentOutputValue> supplier) {
    super(supplier);
    addRequirements(turret);
    m_turret = turret;
  }

  @Override
  public void execute() {
    m_turret.setVoltage(12.0 * m_supplier.get().value);
  }
}
