package frc.robot.subsystems.controlpanel;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private final CANSparkMax m_motor;
  private final CANEncoder m_encoder;
  private CANPIDController m_pidController;
  private int m_targetColor; 

  public ControlPanelSubsystem() {
    m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    m_colorMatcher = new ColorMatcher();
    m_motor = new CANSparkMax(RobotMap.kControlPanelSparkMax, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();

     // Configure motor

 
     m_pidController = m_motor.getPIDController();
 
 
     m_motor.restoreFactoryDefaults();
     m_motor.setIdleMode(IdleMode.kBrake);
 
 
 
     // set PID coefficients
 
     m_pidController.setP(RobotMap.kPanelP);
     m_pidController.setI(RobotMap.kPanelI);
     m_pidController.setD(RobotMap.kPanelD);
     m_pidController.setIZone(RobotMap.kPanelIzone);
     m_pidController.setFF(RobotMap.kPanelFF);
     m_pidController.setOutputRange(RobotMap.kMinOutput, RobotMap.kMaxOutput);
  }

  // Closed position loop using number of rotations as the setpoint
  public void runPositionLoop(double rotations) {
    m_pidController.setReference(rotations, ControlType.kPosition);
  }

  // Rotate the control panel the number of specified segments
  public void rotateSegments(double segments) {

    // Calculate the number of manipulator wheel rotations
    double rotations = (segments * RobotMap.kColorArcLength) / RobotMap.kManipulatorCircumference;

    runPositionLoop(rotations);

  }

  @Override
  public void periodic() {
    Color detectedColor = m_colorSensor.getColor();
    m_matchedColor = m_colorMatcher.getMatchedColor(detectedColor);
    SmartDashboard.putString("Matched Color", m_matchedColor.name());
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);

    m_targetColor = getTargetColor();
  }

  public ControlPanelColor getMatchedColor() {
    return m_matchedColor;
  }
  


  public void rotateToColor() {
    // Got a color
    double segments = (getMatchedColor().ordinal() - m_targetColor) + 2;
    if (segments == 3) {segments = -1;}

    // Rotate the control panel to the target color
    rotateSegments(segments);
  }

  public int getTargetColor() {
    String gameData;
    int targetColor = -1;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
          //Blue case code
          targetColor = ControlPanelColor.BLUE.ordinal();
          break;
        case 'G' :
          //Green case code
          targetColor = ControlPanelColor.GREEN.ordinal();
          break;
        case 'R' :
          //Red case code
          targetColor = ControlPanelColor.RED.ordinal();
          break;
        case 'Y' :
          //Yellow case code
          targetColor = ControlPanelColor.YELLOW.ordinal();
          break;
        default :
          //This is corrupt data
          targetColor = -1;
          break;
      }
    } else {
      //Code for no data received yet
      targetColor = -1;
    }
    return targetColor;
  }
}
