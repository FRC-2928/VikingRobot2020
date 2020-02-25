package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  private CANSparkMax m_climberMotor; //new 
  private Solenoid m_ratchetSolenoid; //new 
  private CANEncoder m_nativeEncoder; //new 

  private ClimberState m_climberState;
  private double m_setpointTarget;

  //Statemachine for overall climber state
  public enum ClimberState{
    STOWED, READY_TO_LATCH, LATCHED, LOWERING, INTERRUPTED, DEPLOYING, ASSENT_COMPLETE, DEPLOYED;
  }



  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public ClimberSubsystem() {
    new CANSparkMax (RobotMap.kClimberMotor, MotorType.kBrushless);
    m_ratchetSolenoid = new Solenoid(RobotMap.kRatchetSolenoid);

    m_climberMotor.restoreFactoryDefaults();

    //These settings are set by default but it's good practice to set them
    m_climberMotor.enableVoltageCompensation(12);
    m_climberMotor.setSmartCurrentLimit(45, 80); //last value should be 0.04? 
    m_climberMotor.setIdleMode(IdleMode.kBrake);
    m_climberMotor.setInverted(false);

    // Initialize the current climber state
    setClimberState(ClimberState.STOWED);

    //Default command will enable brake and stop elevator
    setDefaultCommand(
      new RunCommand(() -> {
        SetElevatorPower(0);
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
    }
  }

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

  // -----------------------------------------------------------
  // Actuator Output
  // -----------------------------------------------------------
  public void SetElevatorPower(double power){
    m_climberMotor.set(power);
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
  // Sensor Input
  // -----------------------------------------------------------
  public double getElevatorNativeEncoder(){
    m_nativeEncoder = m_climberMotor.getEncoder();
    return  m_nativeEncoder.getPosition();
  }

  // Returns position in meters.
  public double getElevatorPosition(){
    double position = getElevatorNativeEncoder() / ClimberConstants.kClimberEncoderTicksPerRotation;
    position /= ClimberConstants.kClimberGearRatio;
    position *= ClimberConstants.kDistancePerPullyRotation;
    return position;
  }
}