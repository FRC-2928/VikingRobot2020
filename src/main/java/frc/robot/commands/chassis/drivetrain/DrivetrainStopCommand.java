package frc.robot.commands.chassis.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.DrivetrainSubsystem;

public class DrivetrainStopCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrain;

  public DrivetrainStopCommand(DrivetrainSubsystem drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  @Override
  public void execute() {
    m_drivetrain.stop();
  }
}
