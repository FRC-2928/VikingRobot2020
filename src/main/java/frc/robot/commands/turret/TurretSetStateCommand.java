package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.TurretControlState;
import frc.robot.types.TargetEstimate;

//Turret Statemachine
public class TurretSetStateCommand extends CommandBase {
  private TurretSubsystem m_turret;
  private TurretControlState m_desiredState;
  private double m_reference;
  private TargetEstimate m_targetEstimate;

  public TurretSetStateCommand(TurretSubsystem turret, TurretControlState desiredState, double reference,
  TargetEstimate targetEstimate) {
    addRequirements(turret);

    m_turret = turret;
    m_desiredState = desiredState;
    m_reference = reference;
    m_targetEstimate = targetEstimate;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.setTurretState(m_desiredState, m_reference, m_targetEstimate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_turret.setTurretState(TurretControlState.IDLE, 0, new TargetEstimate(0, 0, false));
    m_turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
