package frc.robot.subsystems.controlpanel;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.SmartSubsystem;
import frc.robot.types.ControlPanelColor;
import frc.robot.utilities.ColorMatcher;

/**
 * ControlPanelSubsystem handles the control panel manipulator and sensor.
 */
public class ControlPanelSubsystem extends SubsystemBase implements SmartSubsystem{

  private CANSparkMax m_motor;
  private CANPIDController m_motorPID;
  private CANEncoder m_motorEncoder;
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatcher m_colorMatcher;
  private ControlPanelColor m_matchedColor;

  private double m_targetRotations;
 
  //----------------------------------------------------
  //Initization
  //----------------------------------------------------
  public ControlPanelSubsystem() {
    m_motor = new CANSparkMax(RobotMap.kControlPanelSparkMax, MotorType.kBrushless);

    // Config motor
    m_motor.restoreFactoryDefaults();
    m_motor.enableVoltageCompensation(12);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(35, 45, 0);
    m_motor.setInverted(false);

    // Setup encoder and PID
    m_motorEncoder = m_motor.getEncoder();
    m_motorPID = m_motor.getPIDController(); 

    m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    m_colorMatcher = new ColorMatcher();

     // set PID coefficients
     configPanelFeedbackGains();
 
  } 

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    Color detectedColor = m_colorSensor.getColor();
    m_matchedColor = m_colorMatcher.getMatchedColor(detectedColor);
    SmartDashboard.putString("Matched Color", m_matchedColor.name());
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
  }

  // Gets the detected and target color and rotates to target color
  public void rotateToColor() {

    // Get the matched and target color. Add 2 segments to move it to position
    double segments = (getMatchedColor().ordinal() - getTargetColor().ordinal()) + 2;
    // Faster to spin in reverse if segments equals 3
    if (segments == 3) {segments = -1;}

    // Rotate the control panel to the computed number of segment
    SmartDashboard.putNumber("Segment rotation ", segments);
    rotateSegments(segments);
  } 
  
  // Get the target color from the game field
  public ControlPanelColor getTargetColor() {

    String gameData;
    ControlPanelColor targetColor = ControlPanelColor.UNKNOWN;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
          //Blue case code
          targetColor = ControlPanelColor.BLUE;
          break;
        case 'G' :
          //Green case cod
          targetColor = ControlPanelColor.GREEN;
          break;
        case 'R' :
          //Red case code
          targetColor = ControlPanelColor.RED;
          break;
        case 'Y' :
          //Yellow case code
          targetColor = ControlPanelColor.YELLOW;
          break;
        default :
          //This is corrupt data
          break;
      }
    } else {
      //Code for no data received yet
      targetColor = ControlPanelColor.UNKNOWN;
    }
    SmartDashboard.putString("Target Color ", targetColor.name());
    return targetColor;
  }
  
  // Returns whether a color is known
  public BooleanSupplier unknownColor() {

    BooleanSupplier sup = () -> false;
    if (m_matchedColor == ControlPanelColor.UNKNOWN) {
      sup = () -> true;
    }
    return sup;
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  
  
  public void setPosition(double position){

  }
  public void setVelocity(double velocity){

  }
  public void setMotion(double position){

  }
  public void stop(){
    setPower(0);
  }

  // Send a percent power to the motor
  public void setPower(double power) {
    m_motorPID.setReference(power, ControlType.kDutyCycle);
  }

  // Closed position loop using number of rotations as the setpoint
  public void runRotationLoop(double rotations) {

    // Raw rotations
    double setpoint = (rotations * ControlPanelConstants.kGearRatio) + m_motorEncoder.getPosition();
    
    m_motorPID.setReference(setpoint, ControlType.kPosition);
  }

  // Rotate the control panel the number of specified segments
  public void rotateSegments(double segments) {
    // Calculate the number of manipulator wheel rotations
    m_targetRotations = (segments * ControlPanelConstants.kColorArcLength) / ControlPanelConstants.kManipulatorCircumference;

    runRotationLoop(m_targetRotations);
  }

  // Convenience method for running inline command
  public void rotateHalfSegment() {
    rotateSegments(0.5);
  }
    
  //--------------------------------------------------------------
  // Sensor I/O
  //--------------------------------------------------------------

  public double getPosition(){
    return 0;
  }
  public double getVelocity(){
    return 0;
  }
  
  // Return true if we're within 5 degrees
  public boolean atReference() {
    double currentRotations = getMotorRotations();
    double degrees = 1/72; // 5 degrees
    if (m_targetRotations - currentRotations <= degrees) {
      return true;
    } else {
      return false;
    }
  }

  // Get the color detected by the color sensor
  public ControlPanelColor getMatchedColor() {
    return m_matchedColor;
  }

  // Converts encode ticks back into rotations
  public double getMotorRotations(){
    return m_motorEncoder.getPosition();
  }

  // Gets rotations of the actual manipulator wheel
  public double getWheelRotations(){
    return getMotorRotations() / ControlPanelConstants.kGearRatio;
  }

  // Set encoders to zero
  void zeroSensors() {
    m_motorEncoder.setPosition(0);
  }  
    
  //--------------------------------------------------------------
  //Testing/config methods
  //--------------------------------------------------------------
  public void configPanelFeedbackGains() {

    // Grabs the PIDF values from Smartdashboard/Shuffboard
    // m_motorPID.setP(kP, 0);
    // m_motorPID.setI(0, 0);
    // m_motorPID.setIZone(0, 0);
    // m_motorPID.setD(kD, 0);

  }

}
