package frc.robot.commands.shooter.flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.FlywheelSubsystem;

public class FlywheelSetPercentOutputCommand extends CommandBase {
  private FlywheelSubsystem m_flywheel;
  private DoubleSupplier m_supplier;

  public FlywheelSetPercentOutputCommand(FlywheelSubsystem flywheel, double percentOutput) {
    this(flywheel, () -> percentOutput);
  }

  public FlywheelSetPercentOutputCommand(FlywheelSubsystem flywheel, DoubleSupplier supplier) {
    addRequirements(flywheel);
    m_flywheel = flywheel;
    m_supplier = supplier;
  }

  @Override
  public void execute() {
    m_flywheel.setVoltage(12.0 * m_supplier.getAsDouble());
  }
}
