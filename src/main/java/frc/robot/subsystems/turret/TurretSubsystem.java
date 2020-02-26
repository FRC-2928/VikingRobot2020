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
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.SmartSubsystem;
import frc.robot.types.LimelightData;
import frc.robot.types.TargetEstimate;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.Pigeon;
import frc.robot.utilities.Limelight.Limelights;

/**
 * TurretSubsystem is responsible for subsystem level logic with the turret.
 * Positive power/encoder values is left, negative is right
 */
public class TurretSubsystem extends SubsystemBase implements SmartSubsystem{
  private CANSparkMax m_turretMotor;
  private CANEncoder m_motorEncoder;
  private CANPIDController m_turretPID;

  private Pigeon m_pigeon;
  private Limelight m_limelight;
  private LimelightData m_limelightData;

  private TurretState m_turretState;
  private TurretSafetyRangeState m_turretSafetyRangeState;

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
    NORMAL, CORRECTING;
  }

  private double m_setpoint;

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

    m_motorEncoder = m_turretMotor.getEncoder();
    m_turretPID = m_turretMotor.getPIDController();
    m_pigeon = new Pigeon();
    m_limelight = new Limelight(Limelights.TURRET);

    resetTurretEncoder();
    m_correctionReference = 0;
    m_turretSafetyRangeState = TurretSafetyRangeState.NORMAL;
    m_turretState = TurretState.IDLE;

    setDefaultCommand(new RunCommand(this::stop, this));
    // setDefaultCommand(new RunCommand(() -> {
    //   setTurretState(TurretControlState.IDLE, 0, new TargetEstimate(0, 0, false));
    // }, this));

    SmartDashboard.putNumber("Robot Start Angle", 0);
  }

  public void resetTurretEncoder() {
    m_motorEncoder.setPosition(0);
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
    if (m_turretSafetyRangeState == TurretSafetyRangeState.NORMAL) {
      if (degrees < rightMaxLimit) {
        m_turretSafetyRangeState = TurretSafetyRangeState.CORRECTING;
        m_turretState = TurretState.CORRECTING_RANGE;
        m_correctionReference = rightMaxLimit + 360;
        setPosition(m_correctionReference);
        // setValidAngle(m_correctionReference);
      }

      else if (degrees > leftMaxLimit) {
        m_turretSafetyRangeState = TurretSafetyRangeState.CORRECTING;
        m_turretState = TurretState.CORRECTING_RANGE;
        m_correctionReference = leftMaxLimit - 360;
        setPosition(m_correctionReference);
        // setValidAngle(m_correctionReference);
      }
    }

    // Checks if we've finished correcting
    if (m_turretSafetyRangeState == TurretSafetyRangeState.CORRECTING
        || m_turretSafetyRangeState == TurretSafetyRangeState.CORRECTING) {
      if (Math.abs(degrees - m_correctionReference) <= 10) {
        m_turretSafetyRangeState = TurretSafetyRangeState.NORMAL;
        correctTurretCommand.cancel();
      }
    }
  }

  // Field relative turret tracking, depends on starting position
  public void searchForTarget() {
    double reference = (-m_robotYaw % 360) - m_robotStartAngle;
    setPosition(reference);
    m_turretState = TurretState.SEARCHING_FIELD;
    // setValidAngle(reference);
  }

  //Main state setter for Turret, feed a targetEstimate for vision tracking
  public void setTurretState(TurretControlState desiredState, double reference, TargetEstimate targetEstimate) {
    double visionReference = getTurretDegrees() - m_limelightData.getHorizontalOffset();
    SmartDashboard.putString("Turret Desired State", desiredState.toString());
    boolean isTargetFound = m_limelightData.getTargetFound();

    switch (desiredState) {
    case IDLE:
      stop();
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
      if(atReference()){
        m_turretState = TurretState.AT_REFERENCE;
      }
      else{
        m_turretState = TurretState.MOVING_TO_REFERENCE;
      }
      setValidAngle(reference);
      break;

    case VISION_TRACKING:
      if(isTargetFound){
        if(atReference()){
          m_turretState = TurretState.AT_REFERENCE;
        }
        else{
          m_turretState = TurretState.MOVING_TO_REFERENCE;
        }
        setValidAngle(visionReference);
        setPosition(visionReference);
      }
      else{
        if(targetEstimate.isValid()){
          m_turretState = TurretState.GHOSTING_TARGET;
          reference = targetEstimate.getAngle();
          setPosition(reference);
          // setValidAngle(reference);
        }
        else{
          // m_turretState = TurretState.SEARCHING_FIELD;
          searchForTarget();
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

  // Takes a reference and uses checkValidAngle to create a valid path
  // See @checkValidAngle()
  public void setValidAngle(double reference){
    double newReference = checkValidAngle(reference);
    if(newReference != reference && !clearedSafetyRange()){
      m_turretSafetyRangeState = TurretSafetyRangeState.CORRECTING;
    }
    else{
      m_turretSafetyRangeState = TurretSafetyRangeState.NORMAL;
    }
    setPosition(newReference);
  }

  public TurretState getTurretState() {
    return m_turretState;
  }

  public TurretSafetyRangeState getTurretRangeState() {
    return m_turretSafetyRangeState;
  }

  // -----------------------------------------------------------
  // Control Input
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

  // public void setPower(double power) {
  //   m_setpoint = power;
  //   if(inSafetyRange()){
  //     m_turretPID.setReference(power, ControlType.kDutyCycle);
  //   }
  //   else{
  //     correctTurretRange();
  //   } 
  //   m_turretState = TurretState.MANUAL;
  // }

  // Set the rotation degrees of the turret.
  // public void setPosition(double degrees) {
  //   m_setpoint = degrees;

  //   double newReference = checkValidAngle(degrees);
  //   if(newReference != degrees && !clearedSafetyRange()){
  //     m_turretSafetyRangeState = TurretSafetyRangeState.CORRECTING;
  //   }
  //   else{
  //     m_turretSafetyRangeState = TurretSafetyRangeState.NORMAL;
  //   }

  //   kF = SmartDashboard.getNumber("Turret kF", kF);

  //   if (degrees <= 0) {
  //     kF = -kF;
  //   }

  //   m_turretPID.setReference(degreesToMax(degrees), ControlType.kPosition, 0, kF);
  // }

  public void setVelocity(double velocity) {
  }

  public void setMotion(double position) {
  }

  public void stop() {
    m_turretPID.setReference(0, ControlType.kDutyCycle);
    m_turretState = TurretState.IDLE;
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------

  public double getNativeEncoderTicks() {
    return m_motorEncoder.getPosition();
  }

  public double getTurretPosition() {
    return getNativeEncoderTicks() / ConversionConstants.kTurretGearRatio;
  }

  public double getTurretDegrees() {
    return maxToDegrees(getNativeEncoderTicks());
  }

  /**
   * Returns the angle of the turret relative to the field. 0 degrees is facing
   * opponent's alliance stations.
   */
  public double getTurretFieldDegrees() {
    return (m_robotYaw % 360) + getTurretDegrees();
  }

  public LimelightData getLimelightData() {
    return m_limelightData;
  }

  public double getPosition() {
    return maxToDegrees(getNativeEncoderTicks());
  }

  public double getVelocity() {
    return 0;
  }


  //Checks if we're within the error threshold of our reference
  public boolean atReference(){
    if(Math.abs(m_setpoint - getTurretDegrees()) < TurretConstants.kTurretErrorThreshold){
      m_turretState = TurretState.AT_REFERENCE;
      return true;
    }
    m_turretState = TurretState.MOVING_TO_REFERENCE;
    return false;
  }

  //Checks if more than 180 degrees from limits
  public boolean clearedSafetyRange(){
    if(Math.abs(getTurretDegrees() - leftMaxLimit) > 180){
      return true;
    }
    return false;
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
