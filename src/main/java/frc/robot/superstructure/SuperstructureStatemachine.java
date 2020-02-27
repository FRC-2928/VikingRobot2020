package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.controlpanel.RotateSegments;
import frc.robot.commands.controlpanel.RotateToColor;
import frc.robot.commands.intake.FastForwardFeeder;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.commands.shooter.SetShooter.ShooterSetpoint;
import frc.robot.commands.turret.TurretSetPosition;
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

  //Current Superstructure state
  private SuperstructureState m_currentState;

  private double m_turretReference;
  private ShooterSetpoint m_shooterReference;

  private Command m_feedCommand;
  
  public enum SuperstructureControlState{
    IDLE, MANUAL_CONTROL, SETPOINT_SHOOTING, OPEN_LOOP_TURRET, TRACK_TARGET, VISION_SHOOTING,
    CLIMBING, CONTROL_PANEL_ROTATION, CONTROL_PANEL_POSITION;
  }

  public enum SuperstructureState{
    IDLE, MANUAL_OVERRIDE,TRACKING_TARGET, SHOOTING_INIT, SHOOTING, CLIMBING_INIT, CLIMBING, CONTROL_PANEL_INIT,
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

    m_shooterReference = ShooterSetpoint.CLOSE_TRENCH;
    m_turretReference = 0;

    m_feedCommand = new FastForwardFeeder(m_feeder, m_hood, m_flywheel);
  }

  public void setSuperstructureState(SuperstructureControlState desiredState){
    boolean readyToShoot = false;
    updateSubsystemStates();

    switch(desiredState){
      case IDLE:
      //Default commands for subsystems are already idle
      m_currentState = SuperstructureState.IDLE;
      break;

      case MANUAL_CONTROL:
      new ParallelCommandGroup(
        new TurretSetStateCommand(m_turret, TurretControlState.OPEN_LOOP, 0, new TargetEstimate(0, 0, false)),
        new SetShooter(m_flywheel, m_hood, m_shooterReference)
      );
      if(shooterAtReference()){
        readyToShoot = true;
      }
      else{
        m_currentState = SuperstructureState.MANUAL_OVERRIDE;
      }
      break;

      case SETPOINT_SHOOTING:
      new ParallelCommandGroup(
        new TurretSetStateCommand(m_turret, TurretControlState.VISION_TRACKING, 0, getTargetEstimate()),
        new SetShooter(m_flywheel, m_hood, m_shooterReference)
      );
      if(shooterAtReference()){
        readyToShoot = true;
      }
      else{
        m_currentState = SuperstructureState.MANUAL_OVERRIDE;
      }
      break;

      case OPEN_LOOP_TURRET:
      new ParallelCommandGroup(
        new TurretSetStateCommand(m_turret, TurretControlState.OPEN_LOOP, m_turretReference, new TargetEstimate(0, 0, false))
      );
      m_currentState = SuperstructureState.MANUAL_OVERRIDE;
      break;

      case TRACK_TARGET:
      new ParallelCommandGroup(
        new TurretSetStateCommand(m_turret, TurretControlState.VISION_TRACKING, 0, getTargetEstimate())
      );
      m_currentState = SuperstructureState.TRACKING_TARGET;
      break;

      case VISION_SHOOTING:
      new ParallelCommandGroup(
        new TurretSetStateCommand(m_turret, TurretControlState.VISION_TRACKING, 0, getTargetEstimate()),
        new SetShooter(m_flywheel, m_hood, m_shooterReference)
      );
      if(subsystemsAtReference()){
        readyToShoot = true;
      }
      break;

      case CLIMBING:
      new SequentialCommandGroup(
        new TurretSetPosition(m_turret, TurretConstants.kTurretClimbPosition)
        //Add climb code once done
      );
      if(m_turretState == TurretState.MOVING_TO_REFERENCE){
        m_currentState = SuperstructureState.CLIMBING_INIT;
      }
      //When we're climbing, current state = climbing
      break;

      case CONTROL_PANEL_ROTATION:
      new RotateSegments(m_controlPanel, ControlPanelConstants.kRotationDistance).schedule(true);
      m_currentState = SuperstructureState.SPINNING_CONTROL_PANEL;
      break;

      case CONTROL_PANEL_POSITION:
      new SequentialCommandGroup(
        new TurretSetPosition(m_turret, TurretConstants.kTurretControlPanelPosition),
        new RotateToColor(m_controlPanel)
      );
      if(m_turretState == TurretState.MOVING_TO_REFERENCE){
        m_currentState = SuperstructureState.CONTROL_PANEL_INIT;
      }
      else{
        m_currentState = SuperstructureState.SPINNING_CONTROL_PANEL;
      }
      break;
    }

    if(readyToShoot){
      m_feedCommand.schedule(true);
      m_currentState = SuperstructureState.SHOOTING;
    }
    else{
      m_feedCommand.end(true);
      m_currentState = SuperstructureState.SHOOTING_INIT;
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

  //Used for feeding joystick values for manual override
  public void feedTurretReference(double turretReference){
    m_turretReference = turretReference;
  }

  //Used for feeding shooting setpoints 
  public void feedShooter(ShooterSetpoint shooterReference){
    m_shooterReference = shooterReference;
  }

  private void updateSubsystemStates(){
    m_turretState = m_turret.getTurretState();
    m_flywheelState = m_flywheel.getFlywheelState();
    m_hoodState = m_hood.getHoodState();
  }

  private boolean subsystemsAtReference(){
    if(turretAtReference() && shooterAtReference()){
      return true;
    }
    return false;
  }

  private boolean shooterAtReference(){
    if(flywheelAtReference() && hoodAtReference()){
      return true;
    }
    return false;
  }

  private boolean turretAtReference(){
    if(m_turretState == TurretState.AT_REFERENCE){
      return true;
    }
    return false;
  }

  private boolean flywheelAtReference(){
    if(m_flywheelState == FlywheelState.AT_VELOCITY){
      return true;
    }
    return false;
  }

  private boolean hoodAtReference(){
    if(m_hoodState == HoodState.AT_POSITION){
      return true;
    }
    return false;
  }
}
