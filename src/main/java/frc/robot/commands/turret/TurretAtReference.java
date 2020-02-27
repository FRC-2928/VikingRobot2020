package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.TurretSubsystem;

//Used in command groups to check if turret's at reference
public class TurretAtReference extends CommandBase {
  private TurretSubsystem m_turret;

  public TurretAtReference(TurretSubsystem turret) {
    m_turret = turret;

    //No requirements as this shouldn't interrupt other commands
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    if(m_turret.atReference()){
      return true;
    }
    return false;
  }
}
