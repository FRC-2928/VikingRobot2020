package frc.robot.subsystems.controlpanel;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.types.ControlPanelColor;
import frc.robot.utilities.ColorMatcher;

/**
 * ControlPanelSubsystem handles the control panel manipulator and sensor.
 */
public class ControlPanelSubsystem extends SubsystemBase {


  private final ColorSensorV3 m_colorSensor;
  private final ColorMatcher m_colorMatcher;
  private ControlPanelColor m_matchedColor;

  private final WPI_TalonSRX m_motor;
  private double m_targetRotations;
 
  //----------------------------------------------------
  //Initization
  //----------------------------------------------------
  public ControlPanelSubsystem() {
    m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    m_colorMatcher = new ColorMatcher();
    m_motor = new WPI_TalonSRX(RobotMap.kControlPanelTalonWPI);

    m_motor.configFactoryDefault();

    m_motor.configVoltageCompSaturation(12);
    m_motor.enableVoltageCompensation(true);
    m_motor.configNominalOutputForward(0);
    m_motor.configNominalOutputReverse(0);
    m_motor.configNeutralDeadband(0.01);
    m_motor.setNeutralMode(NeutralMode.Coast);

    //motion magic 
		m_motor.configMotionCruiseVelocity(15000, ControlPanelConstants.kTimeoutMs);
    m_motor.configMotionAcceleration(6000, ControlPanelConstants.kTimeoutMs);
    // How much smoothing [0,8] to use during MotionMagic
	  int m_smoothing = 0;
 
     // set PID coefficients
     configPanelFeedbackGains();
 
    m_motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 35, 0.04));
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

  // Return true if we're within 5 degrees
  public boolean atSetpoint() {
    double currentRotations = getMotorRotations();
    double degrees = 1/72; // 5 degrees
    if (m_targetRotations - currentRotations <= degrees) {
      return true;
    } else {
      return false;
    }
  }

  // -----------------------------------------------------------
  // Actuator Output
  // -----------------------------------------------------------
  
  // Closed position loop using number of rotations as the setpoint
  public void runRotationLoop(double rotations) {

    // Zero the sensors so that the math is easier
    zeroSensors();

    // MotionMagic wants the setpoint in encoder ticks
    double setpoint = rotations * ControlPanelConstants.kPanelEncoderTicksPerRotation;
    
    m_motor.set(ControlMode.MotionMagic, setpoint);
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

  // Get the color detected by the color sensor
  public ControlPanelColor getMatchedColor() {
    return m_matchedColor;
  }

  // Get raw encoder ticks
  public double getNativeEncoderTicks(){
    return m_motor.getSelectedSensorPosition();
  }

  // Converts encode ticks back into rotations
  public double getMotorRotations(){
    double rotations = getNativeEncoderTicks() / ControlPanelConstants.kPanelEncoderTicksPerRotation;
    return rotations;
  }

  // Set encoders to zero
  void zeroSensors() {
    m_motor.getSensorCollection().setQuadraturePosition(0, ControlPanelConstants.kTimeoutMs);
  }  
    
  //--------------------------------------------------------------
  //Testing/config methods
  //--------------------------------------------------------------
  public void configPanelFeedbackGains() {

    // double kP = SmartDashboard.getNumber("Turret kP", kP);
    // double kF = SmartDashboard.getNumber("Turret kF", kF);

    m_motor.config_kP(0, ControlPanelConstants.kPanelP);
    m_motor.config_kI(0, ControlPanelConstants.kPanelI);
    m_motor.config_kD(0, ControlPanelConstants.kPanelD);
    // m_motor.config_IntegralZone(0, ControlPanelConstants.kPanelIzone);
    m_motor.config_kF(0, ControlPanelConstants.kPanelFF);

   // m_motor.setOutputRange(ControlPanelConstants.kMinOutput, ControlPanelConstants.kMaxOutput);
  }

}
