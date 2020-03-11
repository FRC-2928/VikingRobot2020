package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemContainer;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class Drive extends CommandBase {
  DrivetrainSubsystem m_drivetrain;
  double m_throttle;
  double m_turn;
  public Drive(SubsystemContainer subsystems, double throttle, double turn) {
    m_drivetrain = subsystems.drivetrain;
    m_throttle = throttle;
    m_turn = turn;

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(m_throttle, m_turn);
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
