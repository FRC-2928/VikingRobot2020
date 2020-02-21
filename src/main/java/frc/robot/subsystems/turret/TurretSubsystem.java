package frc.robot.subsystems.turret;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.TurretConstants;
import frc.robot.types.LimelightData;
import frc.robot.types.TargetEstimate;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.Pigeon;
import frc.robot.utilities.Limelight.Limelights;

/**
 * TurretSubsystem is responsible for subsystem level logic with the turret.
 * Positive power/encoder values is left, negative is right
 */
public class TurretSubsystem extends SubsystemBase {
  private CANSparkMax m_turretMotor;
  private CANEncoder m_turretEncoder;
  private CANPIDController m_turretPID;

  private Pigeon m_pigeon;
  private Limelight m_limelight;
  private LimelightData m_limelightData;

  private TurretState m_turretState;
  private TurretSafetyRangeState m_turretRangeState;

  private double m_correctionReference;

  private RunCommand correctTurretCommand;

  // Feedback gains
  private double kP = PIDConstants.kPTurret;
  private double kD = PIDConstants.kDTurret;
  // Arbritary feedforward in volts
  private double kF = PIDConstants.kFTurret;

  // Turrent working limits
  private final double leftMaxLimit = TurretConstants.kTurretLeftLimit;
  private final double rightMaxLimit = TurretConstants.kTurretRightLimit;

  // Gyro
  private double m_robotYaw;
  // Start angle relative to the goal (left is positive right is negative)
  private double m_robotStartAngle;

  public enum TurretState {
    IDLE, MANUAL, MOVING_TO_REFERENCE, AT_REFERENCE, SEARCHING_FIELD, GHOSTING_TARGET, CORRECTING_RANGE;
  }

  public enum TurretControlState{
    IDLE, OPEN_LOOP, POSITION_CONTROL, VISION_TRACKING;
  }

  public enum TurretSafetyRangeState {
    NORMAL, CORRECTING_LEFT, CORRECTING_RIGHT;
  }

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public TurretSubsystem() {
    m_turretMotor = new CANSparkMax(RobotMap.kTurretSparkMax, MotorType.kBrushless);

    m_turretMotor.restoreFactoryDefaults();

    m_turretMotor.enableVoltageCompensation(12);
    m_turretMotor.setIdleMode(IdleMode.kBrake);
    m_turretMotor.setSmartCurrentLimit(35, 45, 0);

    m_turretMotor.setInverted(false);

    m_turretEncoder = m_turretMotor.getEncoder();
    m_turretPID = m_turretMotor.getPIDController();
    m_pigeon = new Pigeon();
    m_limelight = new Limelight(Limelights.TURRET);

    resetTurretEncoder();
    m_correctionReference = 0;
    m_turretRangeState = TurretSafetyRangeState.NORMAL;
    m_turretState = TurretState.IDLE;

    setDefaultCommand(new RunCommand(() -> {
      setTurretState(TurretControlState.IDLE, 0, null);
    }, this));

    SmartDashboard.putNumber("Robot Start Angle", 0);
  }

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------

  @Override
  public void periodic() {
    configTurretFeedbackGains();
    m_robotYaw = m_pigeon.getYaw();
    m_robotStartAngle = SmartDashboard.getNumber("Robot Start Angle", 0);
    m_limelightData = m_limelight.getLimelightData();

    SmartDashboard.putNumber("Robot yaw", m_robotYaw);
    SmartDashboard.putNumber("Turret position degrees", getTurretDegrees());
    SmartDashboard.putNumber("Turret Amp Draw", m_turretMotor.getOutputCurrent());
    SmartDashboard.putNumber("Turret Voltage Draw", m_turretMotor.getAppliedOutput() * 12);
    SmartDashboard.putString("Turret State", m_turretState.toString());
    SmartDashboard.putBoolean("Limelight valid target", m_limelightData.getTargetFound());
    SmartDashboard.putNumber("Target distance", m_limelightData.getTargetDistance());
  }

  public boolean inSafetyRange(){
    if(Math.abs(getTurretDegrees()) > leftMaxLimit){
      return false;
    }
    return true;
  }

  // Checks if turret is out of bounds, and corrects it
  public void correctTurretRange() {
    double degrees = getTurretDegrees();

    // Checks if turret is beyond limits, and corrects 360 degrees the opposite way
    if (m_turretRangeState == TurretSafetyRangeState.NORMAL) {
      if (degrees < rightMaxLimit) {
        m_turretRangeState = TurretSafetyRangeState.CORRECTING_RIGHT;
        m_turretState = TurretState.CORRECTING_RANGE;
        m_correctionReference = rightMaxLimit + 360;
        setValidAngle(m_correctionReference);
      }

      else if (degrees > leftMaxLimit) {
        m_turretRangeState = TurretSafetyRangeState.CORRECTING_LEFT;
        m_turretState = TurretState.CORRECTING_RANGE;
        m_correctionReference = leftMaxLimit - 360;
        setValidAngle(m_correctionReference);
      }
    }

    // Checks if we've finished correcting
    if (m_turretRangeState == TurretSafetyRangeState.CORRECTING_LEFT
        || m_turretRangeState == TurretSafetyRangeState.CORRECTING_RIGHT) {
      if (Math.abs(degrees - m_correctionReference) <= 10) {
        m_turretRangeState = TurretSafetyRangeState.NORMAL;
        correctTurretCommand.cancel();
      }
    }
  }

  // Field relative turret tracking, depends on starting position
  public void searchForTarget() {
    double reference = (-m_robotYaw % 360) - m_robotStartAngle;
    setValidAngle(reference);;
  }

  //Main state setter for Turret, feed a targetEstimate for vision tracking
  public void setTurretState(TurretControlState desiredState, double reference, TargetEstimate targetEstimate) {
    double visionReference = getTurretDegrees() - m_limelightData.getHorizontalOffset();
    SmartDashboard.putString("Turret Desired State", desiredState.toString());
    boolean isTargetFound = m_limelightData.getTargetFound();

    switch (desiredState) {
    case IDLE:
      stopMotor();
      m_turretState = TurretState.IDLE;
      break;

    case OPEN_LOOP:
      if(inSafetyRange()){
        setPower(reference);
      }
      else{
        correctTurretRange();
      }
      m_turretState = TurretState.MANUAL;
      break;

    case POSITION_CONTROL:
      if(atReference(reference)){
        m_turretState = TurretState.AT_REFERENCE;
      }
      else{
        m_turretState = TurretState.MOVING_TO_REFERENCE;
      }
      setValidAngle(reference);
      break;

    case VISION_TRACKING:
      if(isTargetFound){
        setValidAngle(visionReference);
        m_turretState = TurretState.AT_REFERENCE;
      }
      else{
        if(!targetEstimate.isEstimateValid() || targetEstimate == null){
          searchForTarget();
          m_turretState = TurretState.SEARCHING_FIELD;
        }
        else{
          setValidAngle(targetEstimate.getAngle());
          m_turretState = TurretState.GHOSTING_TARGET;
        }
      }
      break;
    }
  }
  //Returns an angle which is valid and fastest to reference
  public double checkValidAngle(double reference){
    double currentAngle = getTurretDegrees();
    double newReference = reference % 360;

    //Checks if reference is out of bounds
    if(newReference > leftMaxLimit){
      newReference -= 360;
      return newReference;
    }
    else if(newReference < rightMaxLimit){
      newReference += 360;
      return newReference;
    }

    //Checks if there's a faster way to reference
    if(newReference - currentAngle <= -180){
      if(newReference + 360 < leftMaxLimit){
        newReference = newReference + 360;
      }
    }
    else if(newReference - currentAngle >= 180){
      if(newReference - 360 > rightMaxLimit){
        newReference = newReference - 360;
      }
    }

    return newReference;
  }

  public boolean isAngleValid(double reference){
    if(checkValidAngle(reference) != reference){
      return false;
    }
    return true;
  }

  public void setValidAngle(double reference){
    double newReference = checkValidAngle(reference);
    if(newReference != reference && !atReference(newReference)){
      m_turretState = TurretState.CORRECTING_RANGE;
    }
    setPosition(newReference);
  }

  public TurretState getTurretState() {
    return m_turretState;
  }

  public TurretSafetyRangeState getTurretRangeState() {
    return m_turretRangeState;
  }

  public boolean atReference(double reference){
    if(Math.abs(reference - getTurretDegrees()) < TurretConstants.kTurretErrorThreshold){
      return true;
    }
    return false;
  }

  // -----------------------------------------------------------
  // Actuator Output
  // -----------------------------------------------------------

  public void setPower(double power) {
    m_turretPID.setReference(power, ControlType.kDutyCycle);
  }

  public void setPosition(double degrees) {
    kF = SmartDashboard.getNumber("Turret kF", kF);

    if (degrees <= 0) {
      kF = -kF;
    }

    m_turretPID.setReference(degreesToMax(degrees), ControlType.kPosition, 0, kF);
  }

  public void stopMotor() {
    setPower(0);
  }

  // -----------------------------------------------------------
  // Sensor I/O
  // -----------------------------------------------------------

  public void resetTurretEncoder() {
    m_turretEncoder.setPosition(0);
  }

  public double getTurretNativeEncoder() {
    return m_turretEncoder.getPosition();
  }

  public double getTurretPosition() {
    return getTurretNativeEncoder() / ConversionConstants.kTurretGearRatio;
  }

  public double getTurretDegrees() {
    return maxToDegrees(getTurretNativeEncoder());
  }

  /**
   * Returns the angle of the turret relative to the field. 0 degrees is facing
   * opponent's alliance stations.
   */
  public double getTurretFieldDegrees() {
    return (m_robotYaw % 360) + getTurretDegrees();
  }

  // -----------------------------------------------------------
  // Testing/config methods
  // -----------------------------------------------------------

  // Grabs the PIDF values from Smartdashboard/Shuffboard
  public void configTurretFeedbackGains() {
    m_turretPID.setP(kP, 0);
    m_turretPID.setI(0, 0);
    m_turretPID.setIZone(0, 0);
    m_turretPID.setD(kD, 0);
  }

  // -----------------------------------------------------------
  // Conversion methods
  // -----------------------------------------------------------

  // Used to convert native encoder units to turret degrees
  private double degreesToMax(double degrees) {
    return degrees * ConversionConstants.kTurretGearRatio / ConversionConstants.kTurretDegreesPerRotation;
  }

  private double maxToDegrees(double max) {
    return max * ConversionConstants.kTurretDegreesPerRotation / ConversionConstants.kTurretGearRatio;
  }
}
