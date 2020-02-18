package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;
import frc.robot.utilities.Limelight;

//Turret Statemachine
public class TurretSetStateCommand extends CommandBase {
  private TurretSubsystem m_turret;
  private TurretState m_desiredState;

  public TurretSetStateCommand(TurretSubsystem turret, TurretState desiredState) {
    addRequirements(turret);

    m_turret = turret;
    m_desiredState = desiredState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.setTurretState(m_desiredState);
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
