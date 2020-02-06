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
  private boolean correctionFinished = true;
  private double oldDegrees;

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
    LEFT_LIMIT, CORRECTING_LEFT, CORRECTING_RIGHT, RIGHT_LIMIT, NORMAL;
  }

  public TurretSubsystem() {
    m_turretMotor = new CANSparkMax(RobotMap.kTurretSparkMax, MotorType.kBrushless);

    m_turretMotor.restoreFactoryDefaults();

    m_turretMotor.enableVoltageCompensation(8);
    m_turretMotor.setIdleMode(IdleMode.kBrake);
    m_turretMotor.setSmartCurrentLimit(35, 45, 0);

    m_turretMotor.setInverted(true);

    m_turretEncoder = m_turretMotor.getEncoder();

    m_turretPID = m_turretMotor.getPIDController();

    m_turretEncoder.setPosition(0);
    oldDegrees = 0;
    m_turretRangeState = TurretRangeState.NORMAL;

    setDefaultCommand(new RunCommand(this::stopMotor, this));

    // Used to config PIDF gains
    SmartDashboard.putNumber("Turret Reference", 0);
    SmartDashboard.putNumber("Turret kP", kP);
    SmartDashboard.putNumber("Turret kF", kF);
    SmartDashboard.putNumber("Turret kD", kD);
  }

  @Override
  public void periodic() {
    // should this be getTurretDegrees????
    configTurretFeedbackGains();
    SmartDashboard.putNumber("Turret position degrees", getTurretDegrees());
    SmartDashboard.putNumber("Turret Amp Draw", m_turretMotor.getOutputCurrent());
    SmartDashboard.putNumber("Turret Voltage Draw", m_turretMotor.getAppliedOutput()*12);


    // Report reaching limits
    correctTurretRange();
    
    SmartDashboard.putString("Turret range", m_turretRangeState.toString());
  }

  public void setPower(double power) {
    m_turretPID.setReference(power, ControlType.kDutyCycle);
  }

  public void setPosition(double degrees) {
    kF = SmartDashboard.getNumber("Turret kF", kF);

    if(degrees <= 0){
      kF = -kF;
    }

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

    if(m_turretRangeState == TurretRangeState.NORMAL){
      if (degrees > rightMaxLimit) {
        m_turretRangeState = TurretRangeState.RIGHT_LIMIT;
        return TurretRangeState.RIGHT_LIMIT;
      } 

      else if (degrees < leftMaxLimit) {
        m_turretRangeState = TurretRangeState.LEFT_LIMIT;
        return TurretRangeState.LEFT_LIMIT;
      } 
    }

  return TurretRangeState.NORMAL;
}

  public void correctTurretRange() {    
    getTurretRange();
    double reference = 0;

    if (m_turretRangeState == TurretRangeState.RIGHT_LIMIT) {
      m_turretRangeState = TurretRangeState.CORRECTING_RIGHT;
      correctTurretCommand = new RunCommand(() -> this.setPosition(rightMaxLimit - 360), this);
    } 
    
    else if (m_turretRangeState == TurretRangeState.LEFT_LIMIT) {
      m_turretRangeState = TurretRangeState.CORRECTING_LEFT;
      correctTurretCommand = new RunCommand(() -> this.setPosition(leftMaxLimit + 360), this);
    }

    if(m_turretRangeState == TurretRangeState.CORRECTING_RIGHT){
      reference = rightMaxLimit - 360;
      correctTurretCommand.schedule();
      SmartDashboard.putNumber("Turret Correction Reference", reference);
    }

    else if(m_turretRangeState == TurretRangeState.CORRECTING_LEFT){
      reference = leftMaxLimit + 360;
      correctTurretCommand.schedule();
      SmartDashboard.putNumber("Turret Correction Reference", reference);
    }

    if(m_turretRangeState == TurretRangeState.CORRECTING_LEFT || m_turretRangeState == TurretRangeState.CORRECTING_RIGHT){
      if(Math.abs(getTurretDegrees() - reference) <= 5){
        m_turretRangeState = TurretRangeState.NORMAL;
        correctTurretCommand.cancel();
      }
    }
  }

  // Grabs the PIDF values from Smartdashboard/Shuffboard
  public void configTurretFeedbackGains() {
    kP = SmartDashboard.getNumber("Turret kP", kP);
    kD = SmartDashboard.getNumber("Turret kD", kD);
    

    m_turretPID.setP(kP, 0);
    m_turretPID.setI(0, 0);
    m_turretPID.setIZone(0, 0);
    m_turretPID.setD(kD, 0);

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
