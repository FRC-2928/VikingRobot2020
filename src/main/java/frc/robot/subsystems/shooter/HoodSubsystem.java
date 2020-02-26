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
/**
   * Creates a new HoodSubsystem.
   */
public class HoodSubsystem extends SubsystemBase {
  private CANSparkMax m_hoodMotor;
  private CANEncoder m_encoder;
  private CANPIDController m_pid;

  private HoodState m_currentState;

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
    m_hoodMotor = new CANSparkMax(RobotMap.kHoodTalonSRX, MotorType.kBrushless);

    m_hoodMotor.restoreFactoryDefaults();

    //6 volts is 100% enough
    m_hoodMotor.enableVoltageCompensation(4);

    m_hoodMotor.setIdleMode(IdleMode.kBrake);

    //Setting the upper and lower limits of the hood
    m_hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, 1);
    m_hoodMotor.setSoftLimit(SoftLimitDirection.kForward, 50);

    //Hood literally draws 12 amps at max but better safe than sorry
    m_hoodMotor.setSmartCurrentLimit(8, 8);

    m_encoder = m_hoodMotor.getEncoder();
    m_pid = m_hoodMotor.getPIDController();

    resetHoodEncoder();

    setDefaultCommand(new RunCommand(() -> this.setHoodState(HoodControlState.IDLE, 0), this));

    //Placing the hood gains on ShuffleBoard
    SmartDashboard.putNumber("Hood kP", kP);
    SmartDashboard.putNumber("Hood kD", kD);
    SmartDashboard.putNumber("Hood kF", 0);
  }
  
  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Native Units", m_encoder.getPosition());
    SmartDashboard.putNumber("Hood Position", getHoodDegrees());
    SmartDashboard.putNumber("Hood voltage", m_hoodMotor.getAppliedOutput() * 12);
    SmartDashboard.putNumber("Hood amps", m_hoodMotor.getOutputCurrent());
  }

  public void setHoodState(HoodControlState desiredState, double reference){
    switch(desiredState){
      case IDLE:
      stopHood();
      m_currentState = HoodState.IDLE;
      break;

      case OPEN_LOOP:
      setPower(reference);
      m_currentState = HoodState.MANUAL;
      break;

      case POSITION_CONTROL:
      setHoodDegrees(reference);
      if(atReference(reference)){
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

  public boolean atReference(double reference){
    if(Math.abs(reference - getHoodDegrees()) < HoodConstants.kHoodErrorThreshold){
      return true;
    }
    return false;
  }

  public double getHoodDegrees(){
    return sparkToDegrees(m_encoder.getPosition());
  }

  public void resetHoodEncoder(){
    m_encoder.setPosition(0);
  }

  // -----------------------------------------------------------
  // Testing and Configuration
  // -----------------------------------------------------------
  //For tuning gains, will take out once we've finalized everything
  public void configPIDGains(){

    m_pid.setP(kP, 0);
    m_pid.setI(0, 0);
    m_pid.setIZone(0);
    m_pid.setD(kD, 0);

    System.out.println("Hood configed");
  }

  //In total degrees of the hood, ex 30 degrees is hood all the way down
  public void setHoodDegrees(double reference){
    reference = reference - 30;

    if(reference < 0){
      kF = -kF;
    }

    m_pid.setReference(degreesToSpark(reference), ControlType.kPosition, 0, kF, ArbFFUnits.kVoltage);
  }

  public void setPower(double power){
    m_hoodMotor.set(power);
  }

  public void moveHoodUp(){
    setPower(0.3);
  }

  public void moveHoodDown(){
    setPower(-0.3);
  }

  public void stopHood(){
    m_hoodMotor.set(0);
  }

  private double sparkToDegrees(double spark){
    return spark * 360 / ConversionConstants.kHoodGearRatio;
  }

  private double degreesToSpark(double degrees){
    return degrees / 360 * ConversionConstants.kHoodGearRatio;
  }
}