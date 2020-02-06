package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;

/**
 * ClimberSubsystem encompasses the elevator lift and the gearbox brake.
 * It's deployed to 3 different setpoints and then brought down to climb
 */
public class ClimberSubsystem extends SubsystemBase {
  private WPI_TalonFX m_climberMotor;
  private Solenoid m_climberBrake;
  private Solenoid m_climberTomahawk;

  private BrakeState m_brakeState;
  private ClimberState m_climberState;

  //Statemachine for overall climber state
  public enum ClimberState{
    STOWED, READY_TO_LATCH, LATCHED, LOW, MID, HIGH, CLIMBING, CLIMBED, INTERRUPTED;
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
    m_climberTomahawk = new Solenoid(RobotMap.kClimberTomahawk);

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

    // Put PID gains onto the Dashboard
    SmartDashboard.putNumber("Climber kP", PIDConstants.kClimberP);
    SmartDashboard.putNumber("Climber kF", PIDConstants.kClimberFF);

    //Default command will enable brake and stop elevator
    setDefaultCommand(
      new InstantCommand(() -> {
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

  // Calculates the new setpoint based on the current position of the climber
  public double calculateSetpoint(){

    double setpoint = 0;
    double currentPosition = getElevatorPosition();

    switch (m_climberState) {

      case STOWED:
        setpoint = PIDConstants.kStowedPositionSetpoint - currentPosition;
        break;

      case LOW:
        setpoint = PIDConstants.kLowPositionSetpoint - currentPosition;       
        break;

      case MID:
        setpoint = PIDConstants.kMidPositionSetpoint - currentPosition;
        break;

      case HIGH:
        setpoint = PIDConstants.kHighPositionSetpoint - currentPosition;
        break;

      case CLIMBING:
        setpoint = PIDConstants.kLiftPositionSetpoint;
        break;

      default:
        break;
    }
    return setpoint;
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
  // Sensor Input
  // -----------------------------------------------------------
  public double getElevatorNativeEncoder(){
    return m_climberMotor.getSelectedSensorPosition();
  }

  // Returns position in (meters/inches?). Need value for kDistancePerPullyRotation
  public double getElevatorPosition(){
    double position = getElevatorNativeEncoder() / ConversionConstants.kClimberEncoderTicksPerRotation;
    position /= ConversionConstants.kClimberGearRatio;
    position *= ConversionConstants.kDistancePerPullyRotation;
    return position;
  }

  // -----------------------------------------------------------
  // Actuator Output
  // -----------------------------------------------------------
  public void setElevatorPower(double power){
    m_climberMotor.set(ControlMode.PercentOutput, power);
  }

  public void setElevatorPosition(double position){
    if (position < 0.0) {
      m_climberMotor.setInverted(true);
    } else {
      m_climberMotor.setInverted(false);
    }
    // Always pass it a positive value
    m_climberMotor.set(ControlMode.Position, Math.abs(position));
  }

  public void engageTomahawk(){
    m_climberTomahawk.set(true);
    if (m_climberState == ClimberState.READY_TO_LATCH) {
      m_climberState = ClimberState.LATCHED;
    }
  }

  public void disengageTomahawk(){
    m_climberTomahawk.set(false);
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
  // Testing
  // -----------------------------------------------------------

  //Grabs the PIDF values from Smartdashboard/Shuffboard
  public void configClimberGains(){

    // Only configure kP and kF for now
    double kP = SmartDashboard.getNumber("Climber kP", 0);
    double kF = SmartDashboard.getNumber("Climber kF", 0);

    m_climberMotor.config_kP(0, kP);
    m_climberMotor.config_kI(0, PIDConstants.kClimberI);
    m_climberMotor.config_kD(0, PIDConstants.kClimberD);
    m_climberMotor.config_IntegralZone(0, PIDConstants.kClimberIzone);
    m_climberMotor.config_kF(0, kF);

    System.out.println("Climber gains configured: kP " + kP + "kF " + kF);
  }
}