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
    m_turret.setPosition(getTargetPosition(m_positionSupplier.getAsDouble(), m_turret.getMeasuredPosition()));
  }

  static double getTargetPosition(double angle, double current) {
    angle %= 360.0;
    double other = (angle > 0) ? angle - 360.0 : angle + 360.0;

    if (!withinRange(angle)) {
      return other;
    }
    if (!withinRange(other)) {
      return angle;
    }

    if (Math.abs(angle - current) < Math.abs(other - current)) {
      return angle;
    }
    return other;
  }

  static boolean withinRange(double value) {
    return value <= TurretConstants.kMaxAngle && value >= -TurretConstants.kMaxAngle;
  }
}
