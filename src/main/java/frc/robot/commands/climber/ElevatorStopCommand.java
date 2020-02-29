package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ElevatorSubsystem;

public class ElevatorStopCommand extends CommandBase {
  private ElevatorSubsystem m_elevator;

  public ElevatorStopCommand(ElevatorSubsystem elevator) {
    m_elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    m_elevator.stop();
  }
}
