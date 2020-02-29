package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.types.DistanceMap;
/**
 * Sets flywheel RPM and hood angle
 */
public class SetShooter extends CommandBase {
  private FlywheelSubsystem m_flywheel;
  private HoodSubsystem m_hood;
  private ShooterSetpoint m_shooterReference;

  private double m_flywheelReference;
  private double m_hoodReference;

  private double currentSetpoint;

  public enum ShooterSetpoint{
    WALL, INITIATION_LINE, CLOSE_TRENCH;
  }

  public SetShooter(FlywheelSubsystem flywheel, HoodSubsystem hood, ShooterSetpoint shooterSetpoint) {
    m_flywheel = flywheel;
    m_hood = hood;
    m_shooterReference = shooterSetpoint;

    addRequirements(m_flywheel, m_hood);
  }
  
  private void setShooterSetpoint(){
    double flywheelReference = 0;
    double hoodReference = 0;
    switch(m_shooterReference){
      case WALL:
      flywheelReference = DistanceMap.getInstance().getFlywheelRPM(1);
      hoodReference = DistanceMap.getInstance().getHoodDegrees(1);
      break;

      case INITIATION_LINE:
      flywheelReference = DistanceMap.getInstance().getFlywheelRPM(10);
      hoodReference = DistanceMap.getInstance().getHoodDegrees(10);
      break;

      case CLOSE_TRENCH:
      flywheelReference = DistanceMap.getInstance().getFlywheelRPM(17);
      hoodReference = DistanceMap.getInstance().getHoodDegrees(17);
      break;
    }
    m_flywheelReference = flywheelReference;
    m_hoodReference = hoodReference;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setShooterSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set flywheel speed in RPM
    m_flywheel.setVelocity(m_flywheelReference);
    // Set hood position in degrees
    m_hood.setPosition(m_hoodReference);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Default commands will stop the motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
