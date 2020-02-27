package frc.robot.commands.control;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SmartSubsystem;

public class SetPositionCommand extends CommandBase {
    private SmartSubsystem m_subsystem;
    private DoubleSupplier m_positionSupplier;
    private Boolean m_checkIfFinished;

    public SetPositionCommand(SmartSubsystem subsystem, double position) {
        this(subsystem, () -> position, false);
    }

    public SetPositionCommand(SmartSubsystem subsystem, double position, Boolean checkIfFinished) {
      this(subsystem, () -> position, checkIfFinished);
    }

    public SetPositionCommand(SmartSubsystem subsystem, 
                              DoubleSupplier positionSupplier,
                              Boolean checkIfFinished) {
    addRequirements(subsystem);

    m_subsystem = subsystem;
    m_positionSupplier = positionSupplier;
    m_checkIfFinished = checkIfFinished;
  }

  @Override
  public void execute() {
    m_subsystem.setPosition(m_positionSupplier.getAsDouble());
  }

  public boolean isFinished() {
    if (m_checkIfFinished) {
      return m_subsystem.atReference();
    } 
    return false;
  }
}