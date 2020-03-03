package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterManager;

public class ShooterManagerSetReference extends CommandBase {
  private ShooterManager m_shooterManager;
  private double m_hoodReference;
  private double m_flywheelReference;

  public ShooterManagerSetReference(ShooterManager shootermanager, double hoodReference, double flywheelReference) {
  m_shooterManager = shootermanager;
  addRequirements(m_shooterManager);
  
  m_hoodReference = hoodReference;
  m_flywheelReference = flywheelReference;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterManager.setFlywheelReference(m_flywheelReference);
    m_shooterManager.setHoodReferernce(m_hoodReference);
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
    return true;
  }
}
