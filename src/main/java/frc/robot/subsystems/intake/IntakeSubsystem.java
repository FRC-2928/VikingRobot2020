package frc.robot.subsystems.intake;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.SmartSubsystem;

public class IntakeSubsystem extends SubsystemBase implements SmartSubsystem{
  /**
   * Creates a new intake.
   */

  private Solenoid m_solenoidArm;
  private Solenoid m_solenoidBase;

  private CANSparkMax m_motor;  
  private CANEncoder m_motorEncoder; 
  private CANPIDController m_motorPID;

  public enum IntakeState {
    GROUND_PICKUP, STATION_PICKUP, STOWED;
  }

  private IntakeState currentState;
  private double m_setpoint;

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public IntakeSubsystem() {
    m_solenoidArm = new Solenoid(RobotMap.kIntakeArmSolenoid);
    m_solenoidBase = new Solenoid(RobotMap.kIntakeBaseSolenoid);
    m_motor = new CANSparkMax(RobotMap.kIntakeSparkMax, MotorType.kBrushless);

    //These settings are set by default but it's good practice to set them
    m_motor.restoreFactoryDefaults();
    m_motor.enableVoltageCompensation(12);
    m_motor.setSmartCurrentLimit(25, 35); 
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.setInverted(false);

    // Setup encoder and PID
    m_motorEncoder = m_motor.getEncoder();
    m_motorPID = m_motor.getPIDController();

    currentState = IntakeState.STOWED;

    // Set default command
    setDefaultCommand(new RunCommand(this::stowIntake, this)); 
  }

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void groundPickup() {
    moveIntake(IntakeState.GROUND_PICKUP);
  }

  public void stationPickup() {
    moveIntake(IntakeState.STATION_PICKUP);
  }

  // This is the default command
  public void stowIntake() {
    stopMotor();
    moveIntake(IntakeState.STOWED);
  }

  public void moveIntake(IntakeState state) {

    switch (state) {
      case GROUND_PICKUP: 
        m_solenoidArm.set(true);
        m_solenoidBase.set(true);
      break;

      case STATION_PICKUP: 
        m_solenoidArm.set(false);
        m_solenoidBase.set(true);
      break;

      case STOWED:
        m_solenoidArm.set(false);
        m_solenoidBase.set(true);
      break;

      default:
      break;
      }

      currentState = state;
    }

    public IntakeState getIntakeState(){
      return currentState;
    }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------  
  public void setPower(double power){
    m_motorPID.setReference(power, ControlType.kDutyCycle);
  }

  public void setPosition(double position){
    m_motorPID.setReference(position, ControlType.kPosition, 0, IntakeConstants.kF);
  }

  public void setVelocity(double velocity){
  }

  public void setMotion(double position) {
  }

  public void stop() {
    setPower(0);
  }

  public void startMotor () {
    setPower(0.8);
  }

  public void stopMotor () {
    setPower(0);
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  // Acts as a flywheel so there's no linear or rotational position
  public double getPosition(){
    return 0;
  }

  // TODO compute velocity
  public double getVelocity() {
    return 0;
  }

  public double getCurrentDraw(){
    return m_motor.getOutputCurrent();
  }
  
  public boolean atReference(){
    if(Math.abs(m_setpoint - getPosition()) < IntakeConstants.kClimberErrorThreshold){
      return true;
    }
    return false;
  }

  public double getNativeEncoderTicks(){
    m_motorEncoder = m_motor.getEncoder();
    return  m_motorEncoder.getPosition();
  }
}
