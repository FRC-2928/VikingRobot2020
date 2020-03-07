package frc.robot.commands.shooter.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem.State;
import frc.robot.subsystems.shooter.TurretSubsystem.TrackingType;

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
    m_turret.setTrackingType(TrackingType.Setpoint);
  }

  @Override
  public void execute() {
    double targetPosition = m_positionSupplier.getAsDouble();
    double positionError = Math.abs(targetPosition - m_turret.getMeasuredPosition());
    double speedError = Math.abs(m_turret.getMeasuredVelocity());

    if (positionError < TurretConstants.kAcceptablePositionErrorDeg && speedError < TurretConstants.kAcceptableSpeedErrorDegPerSec) {
      m_turret.setVoltage(0.0);
      m_turret.setState(State.Tracked);
    } else {
      m_turret.setPosition(targetPosition);
      m_turret.setState(State.Tracking);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
    m_turret.setState(State.Idle);
    m_turret.setTrackingType(TrackingType.None);
  }
}
