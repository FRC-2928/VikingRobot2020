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
/**
   * Test FlywheelSubsystem to handle shooting and velocity controls
   */
public class FlywheelSubsystem extends SubsystemBase {
  private TalonFX m_flywheelMotor;
  private double velocity;

  private FlywheelState m_currentState;

  private final double kP = PIDConstants.kFlywheelkP;
  private final double kF = PIDConstants.kFlywheelkF;

  public enum FlywheelState{
    IDLE, MANUAL, SPINNING_UP, AT_VELOCITY;
  }

  public enum FlywheelControlState{
    IDLE, OPEN_LOOP, VELOCITY_CONTROL;
  }

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

   m_flywheelMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 80, 0.04));

   m_flywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
   m_flywheelMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
   m_flywheelMotor.configVelocityMeasurementWindow(64);
   configFeedbackGains();

   setDefaultCommand(new RunCommand(this::stopFlywheel, this));

   SmartDashboard.putNumber("Flywheel kP", kP);
   SmartDashboard.putNumber("Flywheel kF", kF);
   SmartDashboard.putNumber("Target RPM", 0);
   SmartDashboard.putNumber("Shooter Reference", 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel RPM", getFlywheelVelocityRPM());
    SmartDashboard.putNumber("Flywheel native units", m_flywheelMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Flywheel current draw", m_flywheelMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Flywheel Voltage", m_flywheelMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("Flywheel FPS", getFlywheelVelocityFPS());
  }

  public void setFlywheelState(FlywheelControlState desiredState, double reference){
    switch(desiredState){
      case IDLE:
      stopFlywheel();
      m_currentState = FlywheelState.IDLE;
      break;

      case OPEN_LOOP:
      setPower(reference);
      m_currentState = FlywheelState.MANUAL;
      break;

      case VELOCITY_CONTROL:
      setFlywheelRPM(reference);
      if(atReference(reference)){
        m_currentState = FlywheelState.AT_VELOCITY;
      }
      else{
        m_currentState = FlywheelState.SPINNING_UP;
      }
    }
  }

  public FlywheelState getFlywheelState(){
    return m_currentState;
  }

  public void setPower(double power){
    m_flywheelMotor.set(ControlMode.PercentOutput, power);
  }

  public boolean atReference(double reference){
    if(Math.abs(reference - getFlywheelVelocityRPM()) < FlywheelConstants.kFlywheelErrorThreshold){
      return true;
    }
    return false;
  }

  public double getFlywheelVelocityRPM(){
    return fxToRPM(m_flywheelMotor.getSelectedSensorVelocity());
  }

  public double getFlywheelVelocityFPS(){
    return getFlywheelVelocityRPM() * 2 * Math.PI * 2 / 60;
  }

  public void setFlywheelRPM(double rpm){
    rpm = SmartDashboard.getNumber("Shooter Reference", 0);
    m_flywheelMotor.set(ControlMode.Velocity, rpmToFX(rpm));
  }

  public void stopFlywheel(){
    setPower(0);
  }  

  //Temp testing, will take out smartdashboard once fully tuned
  public void configFeedbackGains(){
    double newkP = SmartDashboard.getNumber("Flywheel kP", kP);
    double newkD = SmartDashboard.getNumber("Flywheel kF", kF);

    m_flywheelMotor.config_kP(0, newkP);
    m_flywheelMotor.config_kI(0, SmartDashboard.getNumber("Flywheel kI", 0));
    m_flywheelMotor.config_IntegralZone(0, (int)SmartDashboard.getNumber("Flywheel IntegralZone", 0));
    m_flywheelMotor.config_kD(0, SmartDashboard.getNumber("Flywheel kD", 0));
    m_flywheelMotor.config_kF(0, newkD);

    System.out.println("Flywheel configed yay: kP: " + kP + " kF: " + kF);
  }

  private double rpmToFX(double rpm){
    return rpm*ConversionConstants.kFlywheelEncoderTicksPerRotation * ConversionConstants.kFlywheelGearRatio / 600;
  }

  private double fxToRPM(double fx){
    return fx/ConversionConstants.kFlywheelEncoderTicksPerRotation * 600 / ConversionConstants.kFlywheelGearRatio;
  }
}