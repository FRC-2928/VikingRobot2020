package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.HoodSubsystem;

/*
 * Spins up the flywheel to desired RPM
 */

public class SetHoodDegrees extends CommandBase {

    // The subsystem the command runs on
    private final HoodSubsystem m_hood;
    private double m_hoodDegrees;

    public SetHoodDegrees(HoodSubsystem subsystem, double degrees) {
    m_hood = subsystem;
    m_hoodDegrees = degrees;
    addRequirements(m_hood);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_hood.setHoodDegrees(m_hoodDegrees);
  }

  @Override
  public boolean isFinished() {
    return m_hood.atReference(m_hoodDegrees);
  }

}