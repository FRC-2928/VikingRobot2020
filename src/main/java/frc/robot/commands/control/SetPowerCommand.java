package frc.robot.commands.control;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SmartSubsystem;

public class SetPowerCommand extends CommandBase {
    private SmartSubsystem m_subsystem;
    private DoubleSupplier m_powerSupplier;

    public SetPowerCommand(SmartSubsystem subsystem, double power) {
        this(subsystem, () -> power);
    }

    public SetPowerCommand(SmartSubsystem subsystem, DoubleSupplier powerSupplier) {
    addRequirements(subsystem);

    m_subsystem = subsystem;
    m_powerSupplier = powerSupplier;
  }

  @Override
  public void execute() {
    m_subsystem.setPower(m_powerSupplier.getAsDouble());
  }

  public boolean isFinished() {
      return false;
  }
}