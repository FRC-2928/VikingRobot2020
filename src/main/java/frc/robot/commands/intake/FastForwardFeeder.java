package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class FastForwardFeeder extends CommandBase {
    private FeederSubsystem m_feeder;
    private HoodSubsystem m_hood;
    private FlywheelSubsystem m_flywheel;

    public FastForwardFeeder(FeederSubsystem feeder, HoodSubsystem hood, FlywheelSubsystem flywheel) {
    addRequirements(feeder);
    m_feeder = feeder;
  }

  @Override
  public void execute() {
    m_feeder.fastForwardFeeder();
  }

  public void end() {
    m_flywheel.stop();
    m_hood.stop();
  }
}