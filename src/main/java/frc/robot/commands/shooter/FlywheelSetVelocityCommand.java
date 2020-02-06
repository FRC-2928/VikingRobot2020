package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.FlywheelSubsystem;

public class FlywheelSetVelocityCommand extends CommandBase {
  private FlywheelSubsystem m_flywheel;
  private DoubleSupplier m_velocitySupplier;
  private Trigger m_trigger;

  public FlywheelSetVelocityCommand(FlywheelSubsystem flywheel, double velocityRPM) {
    this(flywheel, () -> velocityRPM);
  }

  public FlywheelSetVelocityCommand(FlywheelSubsystem flywheel, DoubleSupplier velocitySupplier) {
    this(flywheel, velocitySupplier, new Trigger(() -> true));
  }

  public FlywheelSetVelocityCommand(FlywheelSubsystem flywheel, double velocityRPM, Trigger trigger) {
    this(flywheel, () -> velocityRPM, trigger);
  }

  public FlywheelSetVelocityCommand(FlywheelSubsystem flywheel, DoubleSupplier velocitySupplier, Trigger trigger) {
    addRequirements(flywheel);

    m_flywheel = flywheel;
    m_velocitySupplier = velocitySupplier;
    m_trigger = trigger;
  }

  @Override
  public void execute() {
    if (m_trigger.get()) {
      m_flywheel.setVelocity(m_velocitySupplier.getAsDouble());
    } else {
      m_flywheel.stop();
    }
  }
}
