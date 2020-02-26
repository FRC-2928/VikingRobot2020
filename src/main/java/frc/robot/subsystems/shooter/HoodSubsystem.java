package frc.robot.subsystems.shooter;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.SmartSubsystem;
/**
   * Creates a new HoodSubsystem.
   */
public class HoodSubsystem extends SubsystemBase implements SmartSubsystem {
  private CANSparkMax m_motor;  
  private CANEncoder m_motorEncoder; 
  private CANPIDController m_motorPID;

  private HoodState m_currentState;

  // private HoodState m_currentState;
  private double m_setpoint;

  private final double kP = HoodConstants.kP;
  private final double kD = HoodConstants.kD;
  private final double kF = HoodConstants.kF;
  private final double kMaxOutput = HoodConstants.kMaxOutput;
  private final double kMinOutput = HoodConstants.kMinOutput;
  private final double kMaxRPM = HoodConstants.kMaxRPM;
  private final double kMaxVel = HoodConstants.kMaxVel;
  private final double kMinVel = HoodConstants.kMinVel;
  private final double kMaxAcc = HoodConstants.kMaxAcc;
  private final double kAllowedError = HoodConstants.kAllowedError;

  public enum HoodState{
    IDLE, MANUAL, MOVING_TO_POSITION, AT_POSITION;
  }

  public enum HoodControlState{
    IDLE, OPEN_LOOP, POSITION_CONTROL;
  }
  
  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public HoodSubsystem() {
    m_motor = new CANSparkMax(RobotMap.kHoodSparkMax, MotorType.kBrushless);

    // Config motor
    m_motor.restoreFactoryDefaults();
    m_motor.enableVoltageCompensation(12);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(35, 45, 0);
    m_motor.setInverted(false);

    // Setup encoder and PID
    m_motorEncoder = m_motor.getEncoder();
    m_motorPID = m_motor.getPIDController(); 

    // Configure Smart Motion 
    m_motorPID.setSmartMotionMaxVelocity(kMaxVel, 0);
    m_motorPID.setSmartMotionMinOutputVelocity(kMinVel, 0);
    m_motorPID.setSmartMotionMaxAccel(kMaxAcc, 0);
    m_motorPID.setSmartMotionAllowedClosedLoopError(kAllowedError, 0);

    // Apply conversions
    // m_motorEncoder.setPositionConversionFactor(factor);

    // Zero encoder and configure PID
    resetEncoder();
    configPIDGains();

    setDefaultCommand(new RunCommand(() -> this.stop(), this));

    //Placing the hood gains on ShuffleBoard
    SmartDashboard.putNumber("Hood kP", kP);
    SmartDashboard.putNumber("Hood kD", kD);
    SmartDashboard.putNumber("Hood kF", 0);
    SmartDashboard.putNumber("Hood Target Degrees", getPosition());
  }
  
  // Reset the encoder
  public void resetEncoder(){
    m_motorEncoder.setPosition(0);
  }

  // -----------------------------------------------------------
  // Periodic Loop
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Position degrees", getPosition());
    SmartDashboard.putBoolean("Hood atReference", atReference());
  }

  public void setHoodState(HoodControlState desiredState, double position){
    switch(desiredState){
      case IDLE:
      stop();
      m_currentState = HoodState.IDLE;
      break;

      case OPEN_LOOP:
      setPower(position);
      m_currentState = HoodState.MANUAL;
      break;

      case POSITION_CONTROL:
      setPosition(position);
      if(atReference()){
        m_currentState = HoodState.AT_POSITION;
      }
      else{
        m_currentState = HoodState.MOVING_TO_POSITION;
      }
    }
  }

  public HoodState getHoodState(){
    return m_currentState;
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  public void setPower(double power){
    m_setpoint = power;
    m_motorPID.setReference(power, ControlType.kDutyCycle);
  }

  //In total degrees of the hood, ex 30 degrees is hood all the way down
  public void setPosition(double position){
    m_setpoint = position;
    if(position < HoodConstants.kHoodLowerLimit){
      position = HoodConstants.kHoodLowerLimit;
      System.out.println("Hood setpoint exceeded lower limit");
    }
    else if(position > HoodConstants.kHoodUpperLimit){
      position = HoodConstants.kHoodUpperLimit;
      System.out.println("Hood setpoint exceeded upper limit");
    }
    position =- 30;
    // TODO position conversions
    m_motorPID.setReference(degreesToSRX(position), ControlType.kPosition, 0, HoodConstants.kF);
    // m_motor.set(ControlMode.Position, degreesToSRX(position));
  }

  public void setVelocity(double velocity){
    m_setpoint = velocity;
    m_motorPID.setReference(velocity, ControlType.kSmartVelocity);
  }

  public void setMotion(double position){
    m_setpoint = position;
    m_motorPID.setReference(position, ControlType.kSmartMotion);
    // m_motor.set(ControlMode.MotionMagic, position);
  }

  public void stop(){
    setPower(0);
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  // Returns rotations for SparkMax
  public double getNativeEncoderTicks() {
    return m_motorEncoder.getPosition();
  }

  // TODO get the conversions
  public double getPosition(){
    return srxToDegrees(getNativeEncoderTicks());
    // return srxToDegrees(m_motor.getSelectedSensorPosition());
  }

  public double getVelocity(){
    return 0;
  }

  public boolean atReference(){
    if(Math.abs(m_setpoint - getPosition()) < HoodConstants.kHoodErrorThreshold){
      return true;
    }
    return false;
  }

  // -----------------------------------------------------------
  // Conversions
  // -----------------------------------------------------------
  private double srxToDegrees(double srx){
    return srx * 360 / ConversionConstants.kHoodEncoderTicksPerRotation / ConversionConstants.kHoodGearRatio;
  }

  private double degreesToSRX(double degrees){
    return degrees / 360 * ConversionConstants.kHoodEncoderTicksPerRotation * ConversionConstants.kHoodGearRatio;
  }

  // -----------------------------------------------------------
  // Testing and Configuration
  // -----------------------------------------------------------
  //For tuning gains, will take out once we've finalized everything
  public void configPIDGains(){
    double newkP = SmartDashboard.getNumber("Hood kP", kP);
    double newkD = SmartDashboard.getNumber("Hood kD", kD);
    double newkF = SmartDashboard.getNumber("Hood kF", kF);

    m_motorPID.setP(newkP, 0);
    m_motorPID.setI(0, 0);
    m_motorPID.setIZone(0, 0);
    m_motorPID.setD(newkD, 0);
    m_motorPID.setFF(newkF);

    m_motorPID.setOutputRange(kMinOutput, kMaxOutput);

    System.out.println("Hood configed");
  }

}
