package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotMap;

/**
 * ClimberSubsystem encompasses the elevator lift and the gearbox brake.
 * It's deployed to 3 different setpoints and then brought down to climb
 */
public class ClimberSubsystem extends SubsystemBase {
  private WPI_TalonFX m_climberMotor;
  private Solenoid m_climberBrake;

  private BrakeState m_brakeState;
  private ClimberState m_climberState;
  private double m_setpointTarget;

  //Statemachine for overall climber state
  public enum ClimberState{
    STOWED, READY_TO_LATCH, LATCHED, LOW, MID, HIGH, LOWERING, INTERRUPTED, DEPLOYING, ASSENT_COMPLETE, DEPLOYED;
  }

  //Statemachine for pneumatic brake in the gearbox
  public enum BrakeState{
    OFF, ON;
  }

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public ClimberSubsystem() {
    m_climberMotor = new WPI_TalonFX(RobotMap.kClimberTalonFX);
    m_climberBrake = new Solenoid(RobotMap.kClimberSolenoidBrake);

    m_climberMotor.configFactoryDefault();

    //These settings are set by default but it's good practice to set them
    m_climberMotor.configVoltageCompSaturation(12);
    m_climberMotor.enableVoltageCompensation(true);
    m_climberMotor.configNominalOutputForward(0);
    m_climberMotor.configNominalOutputReverse(0);
    m_climberMotor.configNeutralDeadband(0.01);
    m_climberMotor.setNeutralMode(NeutralMode.Brake);
    m_climberMotor.setInverted(false);

    m_climberMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 0.04));

    m_climberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // Initialize the current climber state
    setClimberState(ClimberState.STOWED);
    setBrakeState(BrakeState.ON);

    /** How much smoothing [0,8] to use during MotionMagic */
	  int _smoothing = 0;

    // Put PID gains onto the Dashboard
    SmartDashboard.putNumber("Climber kP", ClimberConstants.kClimberP);
    SmartDashboard.putNumber("Climber kF", ClimberConstants.kClimberFF);

    /* Set acceleration and vcruise velocity - see documentation */
    //motion magic 
		m_climberMotor.configMotionCruiseVelocity(15000, ClimberConstants.kClimberTimeout);
		m_climberMotor.configMotionAcceleration(6000, ClimberConstants.kClimberIzone);//place holders

    //Default command will enable brake and stop elevator
    setDefaultCommand(
      new RunCommand(() -> {
        this.setBrakePosition(BrakeState.ON);
        setElevatorPower(0);
      }, this)
    );
  }

  //---------------------------------------------------------
  // Process Logic
  //---------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
  }

  public void deployToTop() {
    double currentPosition = getElevatorPosition();
    m_setpointTarget = ClimberConstants.kDeployedPositionSetpoint;

    // If not deployed to top then go there, otherwise command isFinished
    if (!atSetpoint()) {
      double setpoint = m_setpointTarget - currentPosition;
      setElevatorPosition(setpoint);
    }
  }
/*
  public void deployToLow() {
    double currentPosition = getElevatorPosition();
    m_setpointTarget = ClimberConstants.kLowPositionSetpoint;
    if (!atSetpoint()) {
      double setpoint = m_setpointTarget - currentPosition;
      setElevatorPosition(setpoint);
    }
  }

  public void deployToMid() {
    double currentPosition = getElevatorPosition();
    m_setpointTarget = ClimberConstants.kMidPositionSetpoint;
    if (!atSetpoint()) {
      double setpoint = m_setpointTarget - currentPosition;
      setElevatorPosition(setpoint);
    }  
  }  

  public void deployToHigh() {
    double currentPosition = getElevatorPosition();
    m_setpointTarget = ClimberConstants.kHighPositionSetpoint;
    if (!atSetpoint()) {
      double setpoint = m_setpointTarget - currentPosition;
      setElevatorPosition(setpoint);
    }
  } 
*/
  public boolean atSetpoint() {
    if (m_setpointTarget - getElevatorPosition() <= 0.1) {
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

  public void setBrakeState(BrakeState state) {
    m_brakeState = state;
  }

  // -----------------------------------------------------------
  // Actuator Output
  // -----------------------------------------------------------
  public void setElevatorPower(double power){
    m_climberMotor.set(ControlMode.PercentOutput, power);
  }

  public void setElevatorPosition(double position){
    m_climberMotor.set(ControlMode.MotionMagic, position);
  }

  public void setBrakePosition(BrakeState state){
    m_brakeState = state;

    switch(state){
      case ON:
      setSolenoid(true);
      setBrakeState(BrakeState.ON);
      break;

      case OFF:
      setSolenoid(false);
      setBrakeState(BrakeState.OFF);
      break;
    }
  }

  public void setSolenoid(boolean state){
    if(state == true){
      m_climberBrake.set(true);
    }

    if(state == false){
      m_climberBrake.set(false);
    }
  }
  
  // -----------------------------------------------------------
  // Sensor Input
  // -----------------------------------------------------------
  public double getElevatorNativeEncoder(){
    return m_climberMotor.getSelectedSensorPosition();
  }

  // Returns position in meters.
  public double getElevatorPosition(){
    double position = getElevatorNativeEncoder() / ClimberConstants.kClimberEncoderTicksPerRotation;
    position /= ClimberConstants.kClimberGearRatio;
    position *= ClimberConstants.kDistancePerPullyRotation;
    return position;
  }

  // -----------------------------------------------------------
  // Testing and Configuration
  // -----------------------------------------------------------

  //Grabs the PIDF values from Smartdashboard/Shuffboard
  public void configClimberGains(){

    // Only configure kP and kF for now
    double kP = SmartDashboard.getNumber("Climber kP", 0);
    double kF = SmartDashboard.getNumber("Climber kF", 0);

    //may want to adjust the smoothing
    int curveStrength = (int)SmartDashboard.getNumber("curveStrength", 0);

    m_climberMotor.config_kP(0, kP);
    m_climberMotor.config_kI(0, ClimberConstants.kClimberI);
    m_climberMotor.config_kD(0, ClimberConstants.kClimberD);
    m_climberMotor.config_IntegralZone(0, ClimberConstants.kClimberIzone);
    m_climberMotor.config_kF(0, kF);

    System.out.println("Climber gains configured: kP " + kP + "kF " + kF);

    //may want to adjust the smoothing
    m_climberMotor.configMotionSCurveStrength(curveStrength);

 
  }
}