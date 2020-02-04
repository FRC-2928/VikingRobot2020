package frc.robot.subsystems.turret;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;

/**
 * TurretSubsystem is responsible for subsystem level logic with the turret.
 * Positive power/encoder values is right, negative is left
 */
public class TurretSubsystem extends SubsystemBase {
  private CANSparkMax m_turretMotor;
  private CANEncoder m_turretEncoder;
  private CANPIDController m_turretPID;

  private TurretState m_turretState;
  private TurretRangeState m_turretRangeState;

  // Feedback gains
  private double kP = PIDConstants.kPTurret;
  // Arbritary feedforward in volts
  private double kF = PIDConstants.kFTurret;

  // Turrent working limits
  private final double leftMaxLimit = -225;
  private final double rightMaxLimit = 225;

  public enum TurretState {
    IDLE, SEARCHING, FOUND, LOCKED;
  }

  public enum TurretRangeState {
    LEFT_LIMIT, RIGHT_LIMIT, NORMAL;
  }

  public TurretSubsystem() {
    m_turretMotor = new CANSparkMax(RobotMap.kTurretSparkMax, MotorType.kBrushless);

    m_turretMotor.restoreFactoryDefaults();

    m_turretMotor.enableVoltageCompensation(12);
    m_turretMotor.setIdleMode(IdleMode.kBrake);
    m_turretMotor.setSmartCurrentLimit(30, 45, 250);

    m_turretEncoder = m_turretMotor.getEncoder();

    m_turretPID = m_turretMotor.getPIDController();

    // Used to config PIDF gains
    SmartDashboard.putNumber("Turret Reference", 0);
    SmartDashboard.putNumber("Turret kP", kP);
    SmartDashboard.putNumber("Turret kF", kF);
  }

  @Override
  public void periodic() {
    // should this be getTurretDegrees????
    SmartDashboard.putNumber("Turret position degrees", getTurretPosition());

    // Report reaching limits
    m_turretRangeState = getTurretRange();
  }

  public void setPower(double power) {
    m_turretMotor.set(power);
  }

  public void setPosition(double degrees) {
    kF = SmartDashboard.getNumber("Turret kF", kF);

    m_turretPID.setReference(degreesToMax(degrees), ControlType.kPosition, 0, kF);
  }

  public void stopMotor() {
    setPower(0);
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
    return 420; // Placeholder
  }

  public TurretRangeState getTurretRange() {
    double degrees = getTurretDegrees();

    if (degrees > rightMaxLimit) {
      return TurretRangeState.RIGHT_LIMIT;
    } 

    else if (degrees < leftMaxLimit) {
      return TurretRangeState.LEFT_LIMIT;
    } 

    return TurretRangeState.NORMAL;
  }

  public void correctTurretRange() {
    if (m_turretRangeState == TurretRangeState.RIGHT_LIMIT) {
      SmartDashboard.putString("Correcting Turret position from Right Limit", " ");
      setPosition(getTurretDegrees() - 360);
    } 
    
    else if (m_turretRangeState == TurretRangeState.LEFT_LIMIT) {
      SmartDashboard.putString("Correcting Turret position from Left Limit", " ");
      setPosition(getTurretDegrees() + 360);
    }
  }

  // Grabs the PIDF values from Smartdashboard/Shuffboard
  public void configTurretFeedbackGains() {
    kP = SmartDashboard.getNumber("Turret kP", kP);

    m_turretPID.setP(kP, 0);
    m_turretPID.setI(0, 0);
    m_turretPID.setIZone(0, 0);
    m_turretPID.setD(0, 0);

    System.out.println("Turret gains configed: kP " + kP + "kF " + kF);
  }

  public void searchForTarget() {

  }

  public void setTurretState(TurretState state) {
    m_turretState = state;

    switch (state) {
    case IDLE:
      stopMotor();
      break;

    case SEARCHING:
      break;

    case FOUND:
      break;

    case LOCKED:
      break;

    default:
      break;
    }
  }

  public TurretState getTurretState() {
    return m_turretState;
  }

  // Used to convert native encoder units to turret degrees
  private double degreesToMax(double degrees) {
    return degrees * ConversionConstants.kTurretGearRatio / ConversionConstants.kTurretDegreesPerRotation;
  }

  private double maxToDegrees(double max) {
    return max * ConversionConstants.kTurretDegreesPerRotation / ConversionConstants.kTurretGearRatio;
  }
}
