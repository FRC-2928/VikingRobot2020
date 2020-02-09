package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ElevatorSubsystem;

public class ElevatorSetPositionCommand extends CommandBase {
  private ElevatorSubsystem m_elevator;
  private double m_targetPosition;

  public ElevatorSetPositionCommand(ElevatorSubsystem elevator, double position) {
    addRequirements(elevator);

    m_elevator = elevator;
    m_targetPosition = position;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_elevator.setPosition(m_targetPosition);
  }

  @Override
  public boolean isFinished() {
    return m_elevator.atTargetPosition();
  }
}
