package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.TurretSubsystem;

public class TurretSetPositionCommand extends CommandBase {
  private TurretSubsystem m_turret;
  private DoubleSupplier m_positionSupplier;

  public TurretSetPositionCommand(TurretSubsystem turret, double positionDegrees) {
    this(turret, () -> positionDegrees);
  }

  public TurretSetPositionCommand(TurretSubsystem turret, DoubleSupplier positionSupplier) {
    addRequirements(turret);

    m_turret = turret;
    m_positionSupplier = positionSupplier;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_turret.setPosition(m_positionSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
