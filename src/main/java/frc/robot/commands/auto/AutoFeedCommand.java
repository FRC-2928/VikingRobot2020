package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.FeederSubsystem;
import frc.robot.subsystems.indexer.HopperSubsystem;

public class AutoFeedCommand extends CommandBase {
  private HopperSubsystem m_hopper;
  private FeederSubsystem m_feeder;

  private BooleanSupplier m_readyToFeedSupplier;

  public AutoFeedCommand(HopperSubsystem hopper, FeederSubsystem feeder, BooleanSupplier readyToFeedSupplier) {
    addRequirements(hopper, feeder);

    m_hopper = hopper;
    m_feeder = feeder;
    m_readyToFeedSupplier = readyToFeedSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_readyToFeedSupplier.getAsBoolean()) {
      m_hopper.setVoltage(12.0);
      m_feeder.setVoltage(12.0);
    } else {
      m_hopper.stop();
      m_feeder.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
