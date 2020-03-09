package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.SmartSubsystem;
/**
   * Test FlywheelSubsystem to handle shooting and velocity controls
   */
public class FlywheelSubsystem extends SubsystemBase implements SmartSubsystem {
  private TalonFX m_flywheelMotor;

  private double m_setpoint;
  private double m_setpointReference;
  private boolean m_setpointEnabled;

  private FlywheelState m_currentState;

  private final double kP = PIDConstants.kFlywheelkP;
  private final double kF = PIDConstants.kFlywheelkF;


  public enum FlywheelState{
    IDLE, MANUAL, SPINNING_UP, AT_VELOCITY;
  }

  public enum FlywheelControlState{
    IDLE, OPEN_LOOP, VELOCITY_CONTROL;
  }

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public FlywheelSubsystem() {
   m_flywheelMotor = new TalonFX(RobotMap.kFlywheelTalonFX);

   m_flywheelMotor.configFactoryDefault();

   m_flywheelMotor.configVoltageCompSaturation(12);
   m_flywheelMotor.enableVoltageCompensation(true);
   m_flywheelMotor.configNominalOutputForward(0);
   m_flywheelMotor.configNominalOutputReverse(0);
   m_flywheelMotor.configNeutralDeadband(0.01);
   m_flywheelMotor.setNeutralMode(NeutralMode.Coast);

   m_flywheelMotor.setInverted(InvertType.InvertMotorOutput);
   m_flywheelMotor.setSensorPhase(true);

   m_flywheelMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 0.04));

   m_flywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
   m_flywheelMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
   m_flywheelMotor.configVelocityMeasurementWindow(64);

   // Configure PID
   configPIDGains();

   setDefaultCommand(new RunCommand(this::stop, this));

   m_setpointEnabled = false;
   m_setpoint = 0;

   SmartDashboard.putNumber("Flywheel kP", kP);
   SmartDashboard.putNumber("Flywheel kF", kF);
   SmartDashboard.putNumber("Target RPM", 0);
  }

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel RPM", getVelocity());
    SmartDashboard.putNumber("Flywheel current draw", m_flywheelMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Flywheel Voltage", m_flywheelMotor.getMotorOutputVoltage());
    SmartDashboard.putBoolean("Flywheel at Reference", atReference());
  }

  public void setSetpoint(boolean enabled, double setpoint){
    if(enabled == true){
      m_setpointEnabled = true;
      m_setpointReference = setpoint;
    }
  }

  public void setFlywheelState(FlywheelControlState desiredState, double reference){
    switch(desiredState){
      case IDLE:
      stop();
      m_currentState = FlywheelState.IDLE;
      break;

      case OPEN_LOOP:
      setPower(reference);
      m_currentState = FlywheelState.MANUAL;
      break;

      case VELOCITY_CONTROL:
      if(m_setpointEnabled == false){
        setVelocity(reference);
        if(atReference()){
          m_currentState = FlywheelState.AT_VELOCITY;
        }
        else{
          m_currentState = FlywheelState.SPINNING_UP;
        }
      }
      else{
        setVelocity(m_setpointReference);
      }
    }
  }

  public FlywheelState getFlywheelState(){
    return m_currentState;
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  public void setPower(double power){
    m_setpoint = power;
    m_flywheelMotor.set(ControlMode.PercentOutput, power);
  }

  public void setPosition(double position) {
    m_setpoint = position;
    m_flywheelMotor.set(ControlMode.Position, position);
  }

  // Flywheel RPM
  public void setVelocity(double velocity){
    m_setpoint = velocity;
    m_flywheelMotor.set(ControlMode.Velocity, rpmToFX(velocity));
  }

  public void setMotion(double position) {
    m_setpoint = position;
    m_flywheelMotor.set(ControlMode.MotionMagic, position);
  }

  public void stop(){
    m_setpoint = 0;
    setPower(0);
  }  

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  public double getPosition() {
    return 0;
  }

  public boolean atPositionReference(){
    if(Math.abs(m_setpoint - getPosition()) < FlywheelConstants.kFlywheelErrorThreshold){
      return true;
    }
    return false;
  }

  public double getVelocity() {
    return fxToRPM(m_flywheelMotor.getSelectedSensorVelocity());
  }

  public boolean atVelocityReference(){
    if(Math.abs(m_setpoint - getVelocity()) < FlywheelConstants.kFlywheelErrorThreshold){
      return true;
    }
    return false;
  }

  public boolean atReference(){
    if(Math.abs(m_setpoint - getVelocity()) < FlywheelConstants.kFlywheelErrorThreshold){
      return true;
    }
    return false;
  }

  public double getFlywheelVelocityFPS(){
    return getVelocity() * 2 * Math.PI * 2 / 60;
  }

  // -----------------------------------------------------------
  // Conversions
  // -----------------------------------------------------------
  private double rpmToFX(double rpm){
    return rpm*ConversionConstants.kFlywheelEncoderTicksPerRotation * ConversionConstants.kFlywheelGearRatio / 600;
  }

  private double fxToRPM(double fx){
    return fx/ConversionConstants.kFlywheelEncoderTicksPerRotation * 600 / ConversionConstants.kFlywheelGearRatio;
  }

  // -----------------------------------------------------------
  // Configuration and Testing
  // -----------------------------------------------------------
  //Temp testing, will take out smartdashboard once fully tuned
  public void configPIDGains(){
    double newkP = SmartDashboard.getNumber("Flywheel kP", kP);
    double newkD = SmartDashboard.getNumber("Flywheel kF", kF);

    m_flywheelMotor.config_kP(0, newkP);
    m_flywheelMotor.config_kI(0, SmartDashboard.getNumber("Flywheel kI", 0));
    m_flywheelMotor.config_IntegralZone(0, (int)SmartDashboard.getNumber("Flywheel IntegralZone", 0));
    m_flywheelMotor.config_kD(0, SmartDashboard.getNumber("Flywheel kD", 0));
    m_flywheelMotor.config_kF(0, newkD);

    System.out.println("Flywheel configed yay: kP: " + kP + " kF: " + kF);
  }
}