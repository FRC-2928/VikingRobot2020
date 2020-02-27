package frc.robot.subsystems.climber;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.SmartSubsystem;

/**
 * ClimberSubsystem encompasses the elevator lift and the gearbox brake.
 * It's deployed to 3 different setpoints and then brought down to climb
 */
public class ClimberSubsystem extends SubsystemBase implements SmartSubsystem{
  public CANSparkMax m_motor; //new 
  private Solenoid m_ratchetSolenoid; //new 
  private CANEncoder m_motorEncoder; //new 
  private CANPIDController m_motorPID;

  private ClimberState m_climberState;
  private double m_setpointTarget;

  //Statemachine for overall climber state
  public enum ClimberState{
    STOWED, READY_TO_LATCH, LATCHED, LOWERING, INTERRUPTED, DEPLOYING, ASSENT_COMPLETE, DEPLOYED;
  }

  private double m_setpoint;

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public ClimberSubsystem() {
    m_motor = new CANSparkMax (RobotMap.kClimberMotor, MotorType.kBrushless);
    m_ratchetSolenoid = new Solenoid(RobotMap.kRatchetSolenoid);

    //These settings are set by default but it's good practice to set them
    m_motor.restoreFactoryDefaults();
    m_motor.enableVoltageCompensation(12);
    m_motor.setSmartCurrentLimit(45, 80); //last value should be 0.04? 
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(false);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, 10); //Ten is a placeholder
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, 15); //placeholder value

    // Setup encoder and PID
    m_motorEncoder = m_motor.getEncoder();
    m_motorPID = m_motor.getPIDController();

    // Initialize the current climber state
    setClimberState(ClimberState.STOWED);

    //Default command will enable brake and stop elevator
    setDefaultCommand(new RunCommand(this::stop, this));
  }

  //---------------------------------------------------------
  // Process Logic
  //---------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", getPosition());
  }

  public void deployToTop() {
    double currentPosition = getPosition(); 
    m_setpointTarget = ClimberConstants.kDeployedPositionSetpoint;

    // If not deployed to top then go there, otherwise command isFinished
    if (!atSetpoint()) {
      double setpoint = m_setpointTarget - currentPosition;
      setPosition(setpoint);
    }
  }

  public boolean atSetpoint() {
    if (m_setpointTarget - getPosition() <= 0.1) {
      return true;
    } else {
      return false;
    }  
  }
  
  public void setClimberState(ClimberState state) {
    m_climberState = state;
  }

  public ClimberState getClimberState() {
    return m_climberState;
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  public void setPower(double power){
    m_motor.set(power);
  }

  public void setPosition(double position){
    m_motorPID.setReference(position, ControlType.kPosition, 0, ClimberConstants.kF);
  }

  public void setVelocity(double velocity){

  }
  public void setMotion(double position) {

  }
  public void stop() {
    setPosition(0);
  }

  public void setSolenoid(boolean state){
    if(state == true){
      m_ratchetSolenoid.set(true);
    }

    if(state == false){
      m_ratchetSolenoid.set(false);
    }
  }
  
  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  // Returns position in meters.
  public double getPosition(){
    double position = getNativeEncoderTicks() / ClimberConstants.kClimberEncoderTicksPerRotation;
    position /= ClimberConstants.kClimberGearRatio;
    position *= ClimberConstants.kDistancePerPullyRotation;
    return position;
  }

  public double getVelocity() {
    return 0;
  }
  
  public boolean atReference(){
    if(Math.abs(m_setpoint - getPosition()) < ClimberConstants.kClimberErrorThreshold){
      return true;
    }
    return false;
  }

  public double getNativeEncoderTicks(){
    m_motorEncoder = m_motor.getEncoder();
    return  m_motorEncoder.getPosition();
  }

  // -----------------------------------------------------------
  // Testing/config methods
  // -----------------------------------------------------------

  // Grabs the PIDF values from Smartdashboard/Shuffboard
  public void configTurretFeedbackGains() {
    m_motorPID.setP(ClimberConstants.kP, 0);
    m_motorPID.setI(0, 0);
    m_motorPID.setIZone(0, 0);
    m_motorPID.setD(ClimberConstants.kD, 0);
  }

}