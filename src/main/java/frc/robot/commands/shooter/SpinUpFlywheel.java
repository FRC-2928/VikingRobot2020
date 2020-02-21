package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.FlywheelSubsystem;

/*
 * Spins up the flywheel to desired RPM
 */

public class SpinUpFlywheel extends CommandBase {

    // The subsystem the command runs on
    private final FlywheelSubsystem m_flywheel;
    private double m_flywheelRPM;

    public SpinUpFlywheel(FlywheelSubsystem subsystem, double rpm) {
    m_flywheel = subsystem;
    m_flywheelRPM = rpm;
    addRequirements(m_flywheel);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_flywheel.setFlywheelRPM(m_flywheelRPM);
  }

  @Override
  public boolean isFinished() {
    return m_flywheel.atReference(m_flywheelRPM);
  }

}