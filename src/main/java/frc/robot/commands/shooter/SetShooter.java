package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
/**
 * Sets flywheel RPM and hood angle
 */
public class SetShooter extends CommandBase {
  private FlywheelSubsystem m_flywheel;
  private HoodSubsystem m_hood;

  private double m_flywheelReference;
  private double m_hoodReference;
  public SetShooter(FlywheelSubsystem flywheel, HoodSubsystem hood, double flywheelReference, double hoodReference) {
    m_flywheel = flywheel;
    m_hood = hood;

    m_flywheelReference = flywheelReference;
    m_hoodReference = hoodReference;

    addRequirements(m_flywheel, m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheel.setFlywheelRPM(m_flywheelReference);
    m_hood.setHoodDegrees(m_hoodReference);
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
