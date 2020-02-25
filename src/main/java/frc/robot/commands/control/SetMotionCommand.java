package frc.robot.commands.control;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SmartSubsystem;

public class SetMotionCommand extends CommandBase {
    private SmartSubsystem m_subsystem;
    private DoubleSupplier m_positionSupplier;

    public SetMotionCommand(SmartSubsystem subsystem, double position) {
        this(subsystem, () -> position);
    }

    public SetMotionCommand(SmartSubsystem subsystem, DoubleSupplier positionSupplier) {
    addRequirements(subsystem);

    m_subsystem = subsystem;
    m_positionSupplier = positionSupplier;
  }

  @Override
  public void execute() {
    m_subsystem.setMotion(m_positionSupplier.getAsDouble());
  }

  public boolean isFinished() {
      return false;
  }
}