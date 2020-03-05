package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemContainer;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterManager;

public class SetShooter extends CommandBase {
  private FlywheelSubsystem m_flywheel;
  private HoodSubsystem m_hood;
  private ShooterManager m_shooterManager;

  public SetShooter(SubsystemContainer subsystems) {
    m_flywheel = subsystems.flywheel;
    m_hood = subsystems.hood;
    m_shooterManager = subsystems.shooterManager;

    addRequirements(m_flywheel, m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheel.setVelocity(m_shooterManager.getFlywheelReference());
    m_hood.setHoodDegrees(m_shooterManager.getHoodReference());
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
