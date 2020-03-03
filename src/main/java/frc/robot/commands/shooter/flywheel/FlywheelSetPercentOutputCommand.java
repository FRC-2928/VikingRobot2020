package frc.robot.commands.shooter.flywheel;

import java.util.function.Supplier;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.PercentOutputValue;

import frc.robot.subsystems.shooter.FlywheelSubsystem;

public class FlywheelSetPercentOutputCommand extends SuppliedCommand<PercentOutputValue> {
  private FlywheelSubsystem m_flywheel;

  public FlywheelSetPercentOutputCommand(FlywheelSubsystem flywheel) {
    this(flywheel, 0.0);
  }

  public FlywheelSetPercentOutputCommand(FlywheelSubsystem flywheel, double percentOutput) {
    this(flywheel, () -> new PercentOutputValue(percentOutput));
  }

  public FlywheelSetPercentOutputCommand(FlywheelSubsystem flywheel, Supplier<PercentOutputValue> supplier) {
    super(supplier);
    addRequirements(flywheel);
    m_flywheel = flywheel;
  }

  @Override
  public void execute() {
    m_flywheel.setVoltage(12.0 * m_supplier.get().value);
  }
}
