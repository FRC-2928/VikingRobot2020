package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.intake.FeederSubsystem.IndexState;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem.FlywheelState;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem.HoodState;
import frc.robot.subsystems.turret.TargetEstimator;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;
import frc.robot.types.TargetEstimate;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.Limelight.Limelights;

public class SuperstructureStatemachine{
  private TurretSubsystem m_turret;
  private FlywheelSubsystem m_flywheel;
  private HoodSubsystem m_hood;
  private FeederSubsystem m_feeder;

  //For target estimate
  private TargetEstimator m_targetEstimator;
  private DrivetrainSubsystem m_drivetrain;
  private Limelight m_limelight;

  private TurretState m_turretState;
  private FlywheelState m_flywheelState;
  private HoodState m_hoodState;
  private IndexState m_feederState;
  
  public enum SuperstructureControlState{

  }

  public enum SuperstructureState{

  }

  public SuperstructureStatemachine() {
    m_targetEstimator = new TargetEstimator();
    m_drivetrain = new DrivetrainSubsystem();
    m_turret = new TurretSubsystem();
    m_limelight = new Limelight(Limelights.TURRET);
  }

  public void setSuperstructureState(){
    
  }

  public TargetEstimate getTargetEstimate(){
    m_targetEstimator.update(
      m_drivetrain.getPose(),
      m_turret.getTurretFieldDegrees(),
      m_limelight.getLimelightData()
    );
    TargetEstimate targetEstimate = m_targetEstimator.getEstimate();
    return targetEstimate;
  }
}
