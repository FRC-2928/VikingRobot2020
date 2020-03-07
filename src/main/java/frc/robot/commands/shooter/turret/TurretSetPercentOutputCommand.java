package frc.robot.commands.shooter.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem.State;

public class TurretSetPercentOutputCommand extends CommandBase {
  private TurretSubsystem m_turret;
  private DoubleSupplier m_supplier;

  public TurretSetPercentOutputCommand(TurretSubsystem turret, double percentOutput) {
    this(turret, () -> percentOutput);
  }

  public TurretSetPercentOutputCommand(TurretSubsystem turret, DoubleSupplier supplier) {
    addRequirements(turret);
    m_turret = turret;
    m_supplier = supplier;
  }

  @Override
  public void initialize() {
    m_turret.setState(State.OpenLoop);
  }

  @Override
  public void execute() {
    m_turret.setVoltage(12.0 * m_supplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
    m_turret.setState(State.Idle);
  }
}
