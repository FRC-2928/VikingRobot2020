package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TurretStopTracking extends CommandBase {
  TurretSubsystem m_turret;

  public TurretStopTracking(TurretSubsystem turret) {
    m_turret = turret;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
