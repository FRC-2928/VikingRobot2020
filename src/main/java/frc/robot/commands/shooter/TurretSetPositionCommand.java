package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.TurretSubsystem;

public class TurretSetPositionCommand extends CommandBase {
  private TurretSubsystem m_turret;
  private DoubleSupplier m_positionSupplier;

  private boolean m_correcting;
  private double m_correctingPosition;

  public TurretSetPositionCommand(TurretSubsystem turret, double positionDegrees) {
    this(turret, () -> positionDegrees);
  }

  public TurretSetPositionCommand(TurretSubsystem turret, DoubleSupplier positionSupplier) {
    addRequirements(turret);

    m_turret = turret;
    m_positionSupplier = positionSupplier;
  }

  @Override
  public void execute() {
    double targetPostion = m_positionSupplier.getAsDouble();
    if (m_correcting) {
      targetPostion = m_correctingPosition;
      if (m_turret.atTargetPosition()) {
        m_correcting = false;
      }
    }

    if (Math.abs(targetPostion) > 225) {
      targetPostion = Math.IEEEremainder(targetPostion, 360.0);
      m_correcting = true;
      m_correctingPosition = targetPostion;
    }
    m_turret.setPosition(targetPostion);
  }
}
