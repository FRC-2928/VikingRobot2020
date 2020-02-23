package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.commands.turret.TurretSetStateCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.controlpanel.ControlPanelSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.intake.FeederSubsystem.IndexState;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem.FlywheelState;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem.HoodState;
import frc.robot.subsystems.turret.TargetEstimator;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.TurretControlState;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;
import frc.robot.types.TargetEstimate;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.Limelight.Limelights;

public class SuperstructureStatemachine{
  private TurretSubsystem m_turret;
  private FlywheelSubsystem m_flywheel;
  private HoodSubsystem m_hood;
  private FeederSubsystem m_feeder;
  private ClimberSubsystem m_climber;
  private ControlPanelSubsystem m_controlPanel;

  //For target estimate
  private TargetEstimator m_targetEstimator;
  private DrivetrainSubsystem m_drivetrain;
  private Limelight m_limelight;

  //Subsystem statemachines
  private TurretState m_turretState;
  private FlywheelState m_flywheelState;
  private HoodState m_hoodState;
  private IndexState m_feederState;

  //Current Superstructure state
  private SuperstructureState m_currentState;

  private double m_turretReference;
  private ShooterSetpoint m_shooterReference;

  public enum ShooterSetpoint{
    WALL, INITIATION_LINE, CLOSE_TRENCH, FAR_TRENCH;
  }
  
  public enum SuperstructureControlState{
    IDLE, MANUAL_CONTROL, SETPOINT_SHOOTING, OPEN_LOOP_TURRET, TRACK_TARGET, VISION_SHOOTING,
    CLIMBING, CONTROL_PANEL_ROTATION, CONTROL_PANEL_POSITION;
  }

  public enum SuperstructureState{
    IDLE, SHOOTING_INIT, READY_TO_SHOOT, SHOOTING, CLIMBING_INIT, CLIMBING, CONTROL_PANEL_INIT,
    SPINNING_CONTROL_PANEL;
  }

  public SuperstructureStatemachine(  
  TurretSubsystem turret, FlywheelSubsystem flywheel, HoodSubsystem hood,
  FeederSubsystem feeder, ClimberSubsystem climber, ControlPanelSubsystem controlPanel,
  DrivetrainSubsystem drivetrain) {
    m_turret = turret;
    m_flywheel = flywheel;
    m_hood = hood;
    m_feeder = feeder;
    m_climber = climber;
    m_controlPanel = controlPanel;
    m_drivetrain = drivetrain;

    m_targetEstimator = new TargetEstimator();
    m_limelight = new Limelight(Limelights.TURRET);
  }

  public void setSuperstructureState(SuperstructureControlState desiredState){
    switch(desiredState){
      case IDLE:
      //Default commands for subsystems are already idle
      break;

      case MANUAL_CONTROL:
      new ParallelCommandGroup(
        new TurretSetStateCommand(m_turret, TurretControlState.OPEN_LOOP, 0, new TargetEstimate(0, 0, false)),
        new RunCommand(this::setShooterSetpoint)
      );
      break;
    }
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

  public void setShooterReference(ShooterSetpoint reference){
    m_shooterReference = reference;
  }

  private void setShooterSetpoint(){
    double flywheelReference = 0;
    double hoodReference = 0;
    switch(m_shooterReference){
      case WALL:
      flywheelReference = FlywheelConstants.kSetpointWall;
      hoodReference = HoodConstants.kSetpointWall;
      break;

      case INITIATION_LINE:
      flywheelReference = FlywheelConstants.kSetpointInitiationLine;
      hoodReference = HoodConstants.kSetpointInitiationLine;
      break;

      case CLOSE_TRENCH:
      flywheelReference = FlywheelConstants.kSetpointCloseTrench;
      hoodReference = HoodConstants.kSetpointCloseTrench;
      break;

      case FAR_TRENCH:
      flywheelReference = FlywheelConstants.kSetpointFarTrench;
      hoodReference = HoodConstants.kSetpointFarTrench;
      break;
    }
    new SetShooter(m_flywheel, m_hood, flywheelReference, hoodReference).schedule();
  }

  //Used for feeding joystick values for manual override
  public void feedTurretReference(double turretReference){
    m_turretReference = turretReference;
  }

  //Used for feeding shooting setpoints 
  public void feedShooter(ShooterSetpoint shooterReference){
    m_shooterReference = shooterReference;
  }
}
