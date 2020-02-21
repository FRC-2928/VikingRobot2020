package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.TurretSafetyRangeState;
import frc.robot.utilities.Limelight;

public class TurretLimelightSetPosition extends CommandBase {
  private TurretSubsystem m_turret;
  private Limelight m_limelight;
  private double reference;

  public TurretLimelightSetPosition(TurretSubsystem turret, Limelight limelight) {
    addRequirements(turret);
    m_turret = turret;
    m_limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_limelight.updateReadings();
    reference = m_turret.getTurretDegrees() + m_limelight.getHorizontalOffset();
    SmartDashboard.putNumber("Limelight offset", reference);

    if(m_turret.getTurretRangeState() == TurretSafetyRangeState.NORMAL){
      m_turret.setPosition(reference);
    }
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
