package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
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
  public void execute() {
    double targetPostion = m_positionSupplier.getAsDouble() % 360.0;
    if (targetPostion < -TurretConstants.kMaxAngle) {
      targetPostion += 360.0;
    }
    if (targetPostion > TurretConstants.kMaxAngle) {
      targetPostion -= 360.0;
    }
    m_turret.setPosition(targetPostion);
  }
}
