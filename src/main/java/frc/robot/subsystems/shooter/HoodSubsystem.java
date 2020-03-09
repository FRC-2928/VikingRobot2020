package frc.robot.subsystems.shooter;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.SmartSubsystem;
/**
   * Creates a new HoodSubsystem.
   */
public class HoodSubsystem extends SubsystemBase implements SmartSubsystem {
  private CANSparkMax m_motor;  
  private CANEncoder m_encoder; 
  private CANPIDController m_pid;

  private HoodState m_currentState;

  private double m_setpoint;

  private double m_setpointReference;
  private boolean m_setpointEnabled;
  
  private final double kMaxOutput = HoodConstants.kMaxOutput;
  private final double kMinOutput = HoodConstants.kMinOutput;
  private final double kMaxRPM = HoodConstants.kMaxRPM;
  private final double kMaxVel = HoodConstants.kMaxVel;
  private final double kMinVel = HoodConstants.kMinVel;
  private final double kMaxAcc = HoodConstants.kMaxAcc;
  private final double kAllowedError = HoodConstants.kAllowedError;

  private final double kP = PIDConstants.kHoodkP;
  private final double kD = PIDConstants.kHoodkD;
  private double kF = PIDConstants.kHoodkF;

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

    //6 volts is 100% enough
    m_motor.enableVoltageCompensation(4);
    m_motor.setIdleMode(IdleMode.kBrake);

    //Hood literally draws 12 amps at max but better safe than sorry
    m_motor.setSmartCurrentLimit(8, 8);

    m_motor.setInverted(false);

    //Setting the upper and lower limits of the hood
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, 1);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, 50);

    // Setup encoder and PID
    m_encoder = m_motor.getEncoder();
    m_pid = m_motor.getPIDController();

    // Configure Smart Motion 
    // m_pid.setSmartMotionMaxVelocity(kMaxVel, 0);
    // m_pid.setSmartMotionMinOutputVelocity(kMinVel, 0);
    // m_pid.setSmartMotionMaxAccel(kMaxAcc, 0);
    // m_pid.setSmartMotionAllowedClosedLoopError(kAllowedError, 0);

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
  }
  
  // Reset the encoder
  public void resetEncoder(){
    m_encoder.setPosition(0);
  }

  // -----------------------------------------------------------
  // Periodic Loop
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Degrees", getHoodDegrees());
    SmartDashboard.putBoolean("Hood at Reference", atReference());
  }

  public void setSetpoint(boolean enabled, double setpoint){
    if(enabled == true){
      m_setpointEnabled = true;
      m_setpoint = setpoint;
    }
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
      if(m_setpointEnabled == false){
        setPosition(position);
        if(atReference()){
          m_currentState = HoodState.AT_POSITION;
        }
        else{
          m_currentState = HoodState.MOVING_TO_POSITION;
        }
      }
      else{
        setPosition(m_setpointReference);
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
    m_motor.set(power);
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
    m_pid.setReference(degreesToSpark(position), ControlType.kPosition, 0, HoodConstants.kF);
  }

  //In total degrees of the hood, ex 30 degrees is hood all the way down
  public void setHoodDegrees(double reference){
    m_setpoint = reference;

    if(reference < 0){
      kF = -kF;
    }
    m_pid.setReference(degreesToSpark(reference), ControlType.kPosition, 0, kF, ArbFFUnits.kVoltage);
  }

  // Convenience methods
  public void moveHoodUp(){
    setPower(0.3);
  }

  public void moveHoodDown(){
    setPower(-0.3);
  }

  public void stopHood(){
    setPower(0);
  }

  public void stop(){
    setPower(0);
  }

  public void setVelocity(double velocity){
    m_setpoint = velocity;
    m_pid.setReference(velocity, ControlType.kSmartVelocity);
  }

  public void setMotion(double position){
    m_setpoint = position;
    m_pid.setReference(position, ControlType.kSmartMotion);
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  // Returns rotations for SparkMax
  public double getNativeEncoderTicks() {
    return m_encoder.getPosition();
  }

  public double getHoodDegrees(){
    return sparkToDegrees(m_encoder.getPosition());
  }

  public double getPosition(){
    return sparkToDegrees(getNativeEncoderTicks());
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
  private double sparkToDegrees(double spark){
    return (spark * 360) / ConversionConstants.kHoodGearRatio;
  }

  private double degreesToSpark(double degrees){
    return degrees / 360 * ConversionConstants.kHoodGearRatio;
  }

  // -----------------------------------------------------------
  // Testing and Configuration
  // -----------------------------------------------------------
  //For tuning gains, will take out once we've finalized everything
  public void configPIDGains(){
    double newkP = SmartDashboard.getNumber("Hood kP", kP);
    double newkD = SmartDashboard.getNumber("Hood kD", kD);
    double newkF = SmartDashboard.getNumber("Hood kF", kF);

    m_pid.setP(kP, 0);
    m_pid.setI(0, 0);
    m_pid.setIZone(0);
    m_pid.setD(kD, 0);

    // m_pid.setP(newkP, 0);
    // m_pid.setI(0, 0);
    // m_pid.setIZone(0, 0);
    // m_pid.setD(newkD, 0);
    // m_pid.setFF(newkF);

    // m_pid.setOutputRange(kMinOutput, kMaxOutput);

    System.out.println("Hood configed");
  }

}
