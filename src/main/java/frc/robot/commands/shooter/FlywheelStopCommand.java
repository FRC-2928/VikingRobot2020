package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.FlywheelSubsystem;

public class FlywheelStopCommand extends CommandBase {
  private FlywheelSubsystem m_flywheel;

  public FlywheelStopCommand(FlywheelSubsystem flywheel) {
    addRequirements(flywheel);
    m_flywheel = flywheel;
  }

  @Override
  public void execute() {
    m_flywheel.stop();
  }
}
