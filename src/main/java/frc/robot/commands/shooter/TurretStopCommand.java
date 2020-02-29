package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.TurretSubsystem;

public class TurretStopCommand extends CommandBase {
  private TurretSubsystem m_turret;

  public TurretStopCommand(TurretSubsystem turret) {
    addRequirements(turret);
    m_turret = turret;
  }

  @Override
  public void execute() {
    m_turret.stop();
  }
}
