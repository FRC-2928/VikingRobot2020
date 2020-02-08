package frc.robot.commands.intake;

import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.intake.FeederSubsystem.HopperState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class StartFeeder extends CommandBase {
 
  FeederSubsystem m_subsystem;

  public StartFeeder(FeederSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setHopperState(HopperState.STOPPED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.runFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
