package frc.robot.commands.shooter.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem.State;
import frc.robot.subsystems.shooter.TurretSubsystem.TrackingType;

public class TurretStopCommand extends CommandBase {
  private TurretSubsystem m_turret;

  public TurretStopCommand(TurretSubsystem turret) {
    addRequirements(turret);
    m_turret = turret;
  }

  @Override
  public void initialize() {
    m_turret.setState(State.Idle);
    m_turret.setTrackingType(TrackingType.None);
  }

  @Override
  public void execute() {
    m_turret.stop();
  }
}
