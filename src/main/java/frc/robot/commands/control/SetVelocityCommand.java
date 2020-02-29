package frc.robot.commands.control;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SmartSubsystem;

public class SetVelocityCommand extends CommandBase {
    private SmartSubsystem m_subsystem;
    private DoubleSupplier m_velocitySupplier;

    public SetVelocityCommand(SmartSubsystem subsystem, double velocity) {
        this(subsystem, () -> velocity);
    }

    public SetVelocityCommand(SmartSubsystem subsystem, DoubleSupplier velocitySupplier) {
    addRequirements(subsystem);

    m_subsystem = subsystem;
    m_velocitySupplier = velocitySupplier;
  }

  @Override
  public void execute() {
    m_subsystem.setVelocity(m_velocitySupplier.getAsDouble());
  }

  public boolean isFinished() {
      return false;
  }
}