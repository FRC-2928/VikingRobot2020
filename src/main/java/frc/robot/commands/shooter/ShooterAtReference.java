package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

//Used in command groups to check if hood and flywheel are at reference
public class ShooterAtReference extends CommandBase {
  FlywheelSubsystem m_flywheel;
  HoodSubsystem m_hood;
  public ShooterAtReference(FlywheelSubsystem flywheel, HoodSubsystem hood) {
    m_flywheel = flywheel;
    m_hood = hood;

    //No requirements as this shouldn't interrupt any other commands
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
    if(m_flywheel.atReference() && m_hood.atReference()){
      return true;
    }
    return false;
  }
}
