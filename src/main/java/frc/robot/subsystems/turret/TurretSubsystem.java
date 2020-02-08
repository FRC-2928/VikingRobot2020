package frc.robot.subsystems.turret;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private double m_correctionReference;

  private RunCommand correctTurretCommand;

  // Feedback gains
  private double kP = PIDConstants.kPTurret;
  private double kD = PIDConstants.kDTurret;
  // Arbritary feedforward in volts
  private double kF = PIDConstants.kFTurret;

  // Turrent working limits
  private final double leftMaxLimit = -225;
  private final double rightMaxLimit = 225;

  public enum TurretState {
    IDLE, SEARCHING, FOUND, LOCKED;
  }

  public enum TurretRangeState {
    NORMAL, LEFT_LIMIT, RIGHT_LIMIT, CORRECTING_LEFT, CORRECTING_RIGHT;
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

    m_turretMotor.setInverted(true);

    m_turretEncoder = m_turretMotor.getEncoder();

    m_turretPID = m_turretMotor.getPIDController();

    resetTurretEncoder();
    m_correctionReference = 0;
    m_turretRangeState = TurretRangeState.NORMAL;

    setDefaultCommand(new RunCommand(this::stopMotor, this));

    // Used to config PIDF gains
    SmartDashboard.putNumber("Turret Reference", 0);
    SmartDashboard.putNumber("Turret kP", kP);
    SmartDashboard.putNumber("Turret kF", kF);
    SmartDashboard.putNumber("Turret kD", kD);
  }

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------

  @Override
  public void periodic() {
    configTurretFeedbackGains();
    SmartDashboard.putNumber("Turret position degrees", getTurretDegrees());
    SmartDashboard.putNumber("Turret Amp Draw", m_turretMotor.getOutputCurrent());
    SmartDashboard.putNumber("Turret Voltage Draw", m_turretMotor.getAppliedOutput() * 12);

    // Report reaching limits
    correctTurretRange();

    SmartDashboard.putString("Turret range", m_turretRangeState.toString());
  }
 
  //Checks if turret is out of bounds, and corrects it
  public void correctTurretRange() {
    double degrees = getTurretDegrees();

    //Checks if turret is beyond limits, and corrects 360 degrees the opposite way
    if (m_turretRangeState == TurretRangeState.NORMAL) {
      if (degrees > rightMaxLimit) {
        m_turretRangeState = TurretRangeState.CORRECTING_RIGHT;
        m_correctionReference = rightMaxLimit - 360;
        correctTurretCommand = new RunCommand(() -> this.setPosition(m_correctionReference), this);
        correctTurretCommand.schedule();
      }

      else if (degrees < leftMaxLimit) {
        m_turretRangeState = TurretRangeState.CORRECTING_LEFT;
        m_correctionReference = leftMaxLimit + 360;
        correctTurretCommand = new RunCommand(() -> this.setPosition(m_correctionReference), this);
        correctTurretCommand.schedule();
      }
    }

    //Checks if we've finished correcting
    if (m_turretRangeState == TurretRangeState.CORRECTING_LEFT
        || m_turretRangeState == TurretRangeState.CORRECTING_RIGHT) {
      if (Math.abs(degrees - m_correctionReference) <= 5) {
        m_turretRangeState = TurretRangeState.NORMAL;
        correctTurretCommand.cancel();
      }
    }
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
    return 420; // Placeholder
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
