package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.TurretControlState;
import frc.robot.types.TargetEstimate;

public class TurretSetPosition extends CommandBase {
  private TurretSubsystem m_turret;
  private double m_reference;

  public TurretSetPosition(TurretSubsystem turretSubsystem, double reference) {
    m_turret = turretSubsystem;
    m_reference = reference;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_turret.setTurretState(TurretControlState.POSITION_CONTROL, m_reference, new TargetEstimate(0, 0, false));
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
