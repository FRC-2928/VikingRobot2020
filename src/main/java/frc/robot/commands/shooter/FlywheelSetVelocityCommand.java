package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.FlywheelSubsystem;

public class FlywheelSetVelocityCommand extends CommandBase {
  private FlywheelSubsystem m_flywheel;
  private DoubleSupplier m_velocitySupplier;

  public FlywheelSetVelocityCommand(FlywheelSubsystem flywheel, double velocityRPM) {
    this(flywheel, () -> velocityRPM);
  }

  public FlywheelSetVelocityCommand(FlywheelSubsystem flywheel, DoubleSupplier velocitySupplier) {
    addRequirements(flywheel);

    m_flywheel = flywheel;
    m_velocitySupplier = velocitySupplier;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_flywheel.setVelocity(m_velocitySupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
