package frc.robot.commands.shooter.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class HoodStopCommand extends CommandBase {
  private HoodSubsystem m_hood;

  public HoodStopCommand(HoodSubsystem hood) {
    addRequirements(hood);
    m_hood = hood;
  }

  @Override
  public void execute() {
    m_hood.stop();
  }
}
