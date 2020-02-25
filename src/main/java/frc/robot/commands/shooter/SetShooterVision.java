package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.types.LimelightData;

public class SetShooterVision extends CommandBase {
  private FlywheelSubsystem m_flywheel;
  private HoodSubsystem m_hood;
  private LimelightData m_limelightData;
  
  public SetShooterVision(FlywheelSubsystem flywheel, HoodSubsystem hood, LimelightData limelightData) {
    m_flywheel = flywheel;
    m_hood = hood;
    m_limelightData = limelightData;

    addRequirements(m_flywheel, m_hood);
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
    return false;
  }
}
