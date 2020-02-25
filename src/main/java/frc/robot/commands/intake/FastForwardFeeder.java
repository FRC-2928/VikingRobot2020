package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.FeederSubsystem;

public class FastForwardFeeder extends CommandBase {
    private FeederSubsystem m_feeder;

    public FastForwardFeeder(FeederSubsystem feeder) {
    addRequirements(feeder);
    m_feeder = feeder;
  }

  @Override
  public void execute() {
    m_feeder.fastForwardFeeder();
  }
}