package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.FlywheelSubsystem;

public class FlywheelSetVelocityCommand extends CommandBase {
  private FlywheelSubsystem m_flywheel;
  private double m_velocityRPM;

  public FlywheelSetVelocityCommand(double velocityRPM, FlywheelSubsystem flywheel) {
    addRequirements(flywheel);

    m_velocityRPM = velocityRPM;
    m_flywheel = flywheel;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_flywheel.setVelocity(m_velocityRPM);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
