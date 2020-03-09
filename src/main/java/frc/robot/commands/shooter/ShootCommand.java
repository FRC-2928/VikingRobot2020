package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterManager;

public class ShootCommand extends CommandBase {
  FlywheelSubsystem m_flywheel;
  HoodSubsystem m_hood;
  FeederSubsystem m_feeder;
  ShooterManager m_shooterManager;
  Boolean startFeeder; 
  
  public ShootCommand(FlywheelSubsystem flywheel, HoodSubsystem hood, FeederSubsystem feeder, ShooterManager shooterManager) {
    m_flywheel = flywheel;
    m_hood = hood;
    m_feeder = feeder;
    m_shooterManager = shooterManager;

    addRequirements(m_flywheel, m_hood, m_feeder);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startFeeder = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheel.setVelocity(m_shooterManager.getFlywheelReference());
    m_hood.setHoodDegrees(m_shooterManager.getHoodReference());

    if(m_flywheel.atReference() && m_hood.atReference()){
      // startFeeder = true;
      m_feeder.startFeeder();
    }
    else{
      m_feeder.stop();
    }

    // if(startFeeder == true){
    //   m_feeder.startFeeder();
    // }
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
