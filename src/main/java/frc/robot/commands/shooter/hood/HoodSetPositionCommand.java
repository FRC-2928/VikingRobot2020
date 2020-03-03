package frc.robot.commands.shooter.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class HoodSetPositionCommand extends CommandBase {
  private HoodSubsystem m_hood;
  private DoubleSupplier m_positionSupplier;

  public HoodSetPositionCommand(HoodSubsystem hood, double positionDegrees) {
    this(hood, () -> positionDegrees);
  }

  public HoodSetPositionCommand(HoodSubsystem hood, DoubleSupplier positionSupplier) {
    addRequirements(hood);

    m_hood = hood;
    m_positionSupplier = positionSupplier;
  }

  @Override
  public void execute() {
    m_hood.setPosition(m_positionSupplier.getAsDouble());
  }
}
