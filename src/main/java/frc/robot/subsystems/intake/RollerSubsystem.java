package frc.robot.subsystems.intake;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotMap;

public class RollerSubsystem extends SubsystemBase{
  /**
   * Creates a new intake.
   */

  private CANSparkMax m_motor;  
  private CANEncoder m_encoder; 
  private CANPIDController m_motorPID;

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public RollerSubsystem() {
    m_motor = new CANSparkMax(RobotMap.kIntakeSparkMax, MotorType.kBrushless);

    //These settings are set by default but it's good practice to set them
    m_motor.restoreFactoryDefaults();
    m_motor.enableVoltageCompensation(12);
    m_motor.setSmartCurrentLimit(28, 35); 
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.setInverted(false);

    // Setup encoder and PID
    m_encoder = m_motor.getEncoder();
    m_encoder.setVelocityConversionFactor((1.0/IntakeConstants.kIntakeGearRatio));

    m_motorPID = m_motor.getPIDController();
  }

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------  
  public void setPower(double power){
    m_motorPID.setReference(power, ControlType.kDutyCycle);
  }

  public void stop(){
    setPower(0);
  }

  public void startMotor(){
    setPower(0.85);
  }

  public void reverseMotor(){
    setPower(-0.6);
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

  public double getNativeEncoderTicks(){
    return m_encoder.getPosition();
  }
}
