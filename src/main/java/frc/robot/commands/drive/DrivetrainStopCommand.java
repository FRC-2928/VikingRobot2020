package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

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
