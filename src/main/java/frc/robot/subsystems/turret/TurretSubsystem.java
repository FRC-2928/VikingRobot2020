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
import frc.robot.types.LimelightData;
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
  private TurretRangeState m_turretRangeState;

  private double m_correctionReference;
  private double m_setpointReference;

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
    IDLE, SETPOINT, SEARCHING_FIELD, CORRECTING_RANGE, TRACKING_TARGET, TARGET_LOCKED;
  }

  public enum TurretRangeState {
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
    m_turretRangeState = TurretRangeState.NORMAL;
    m_turretState = TurretState.IDLE;

    setDefaultCommand(new RunCommand(() -> {
      this.stopMotor();
      this.m_turretState = TurretState.IDLE;
    }, this));

    // Used to config PIDF gains
    SmartDashboard.putNumber("Turret Reference", 0);
    SmartDashboard.putNumber("Turret kP", kP);
    SmartDashboard.putNumber("Turret kF", kF);
    SmartDashboard.putNumber("Turret kD", kD);
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
    m_setpointReference = SmartDashboard.getNumber("Turret Reference", 0);
    m_limelightData = m_limelight.getLimelightData();

    SmartDashboard.putNumber("Robot yaw", m_robotYaw);
    SmartDashboard.putNumber("Turret position degrees", getTurretDegrees());
    SmartDashboard.putNumber("Turret Amp Draw", m_turretMotor.getOutputCurrent());
    SmartDashboard.putNumber("Turret Voltage Draw", m_turretMotor.getAppliedOutput() * 12);
    SmartDashboard.putString("Turret State", m_turretState.toString());
    SmartDashboard.putBoolean("Limelight valid target", m_limelightData.getTargetFound());
    SmartDashboard.putNumber("Target distance", m_limelightData.getTargetDistance());
  }

  // Checks if turret is out of bounds, and corrects it
  public void correctTurretRange() {
    double degrees = getTurretDegrees();

    // Checks if turret is beyond limits, and corrects 360 degrees the opposite way
    if (m_turretRangeState == TurretRangeState.NORMAL) {
      if (degrees < rightMaxLimit) {
        m_turretRangeState = TurretRangeState.CORRECTING_RIGHT;
        m_turretState = TurretState.CORRECTING_RANGE;
        m_correctionReference = rightMaxLimit + 360;
        setPosition(m_correctionReference);
      }

      else if (degrees > leftMaxLimit) {
        m_turretRangeState = TurretRangeState.CORRECTING_LEFT;
        m_turretState = TurretState.CORRECTING_RANGE;
        m_correctionReference = leftMaxLimit - 360;
        setPosition(m_correctionReference);
      }
    }

    // Checks if we've finished correcting
    if (m_turretRangeState == TurretRangeState.CORRECTING_LEFT
        || m_turretRangeState == TurretRangeState.CORRECTING_RIGHT) {
      if (Math.abs(degrees - m_correctionReference) <= 10) {
        m_turretRangeState = TurretRangeState.NORMAL;
        correctTurretCommand.cancel();
      }
    }
  }

  // Field relative turret tracking, depends on starting position
  public void searchForTarget() {
    double reference = (-m_robotYaw % 360) - m_robotStartAngle;
    setPosition(reference);
  }

  public void setTurretState(TurretState desiredState) {
    double visionReference = getTurretDegrees() - m_limelightData.getHorizontalOffset();
    SmartDashboard.putString("Turret Desired State", desiredState.toString());
    boolean isTargetFound = m_limelightData.getTargetFound();

    correctTurretRange();

    if (m_turretState != TurretState.CORRECTING_RANGE) {
      switch (desiredState) {
      case IDLE:
        stopMotor();
        m_turretState = desiredState;
        break;

      case SETPOINT:
        setPosition(m_setpointReference);
        m_turretState = desiredState;
        break;

      case SEARCHING_FIELD:
        searchForTarget();
        m_turretState = desiredState;
        break;

      case CORRECTING_RANGE:
        break;

      case TRACKING_TARGET:
        if (!isTargetFound) {
          searchForTarget();
          m_turretState = TurretState.SEARCHING_FIELD;
        } else if (Math.abs(visionReference) < TurretConstants.kTurretLockedThreshold) {
          setPosition(visionReference);
          m_turretState = TurretState.TARGET_LOCKED;
        } else if(isTargetFound){
          setPosition(visionReference);
          m_turretState = TurretState.TRACKING_TARGET;
        }
        break;

      case TARGET_LOCKED:
        if (!isTargetFound) {
          searchForTarget();
          m_turretState = TurretState.SEARCHING_FIELD;
        } else if (Math.abs(visionReference) < TurretConstants.kTurretLockedThreshold) {
          setPosition(visionReference);
          m_turretState = TurretState.TARGET_LOCKED;
        } else if(isTargetFound) {
          setPosition(visionReference);
          m_turretState = TurretState.TRACKING_TARGET;
        }
        break;
      }
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

  public TurretState getTurretState() {
    return m_turretState;
  }

  public TurretRangeState getTurretRangeState() {
    return m_turretRangeState;
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
    kP = SmartDashboard.getNumber("Turret kP", kP);
    kD = SmartDashboard.getNumber("Turret kD", kD);

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
