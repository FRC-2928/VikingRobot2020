package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.RobotMap;
/**
* TurretSubsystem is responsible for subsystem level logic with the turret.
*/
public class TurretSubsystem extends SubsystemBase {
  private WPI_TalonSRX m_turretMotor;
  private TurretState m_turretState;

  //Feedback gains
  private double kP = 0;
  private double kF = 0;

  //Turrent working limits
  private final double minWorkingLimit = -225;
  private final double maxWorkingLimit = 225;

  public enum TurretState{
    IDLE, SEARCHING, FOUND, LOCKED;
  }

  public enum TurretRangeState{
    OVER, UNDER, NORMAL;
  }

  public TurretSubsystem() {
    m_turretMotor = new WPI_TalonSRX(RobotMap.kTurretTalonSRX);

    m_turretMotor.configFactoryDefault();

    //These are set by default, but its good to set them anyways
    m_turretMotor.configVoltageCompSaturation(12);
    m_turretMotor.enableVoltageCompensation(true);
    m_turretMotor.configNominalOutputForward(0);
    m_turretMotor.configNominalOutputReverse(0);
    m_turretMotor.configNeutralDeadband(0.01);

    m_turretMotor.setNeutralMode(NeutralMode.Brake);

    m_turretMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 45, 20));

    m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    //Used to config PIDF gains
    SmartDashboard.putNumber("Turret kP", kP);
    SmartDashboard.putNumber("Turret kF", kF);
  }

  @Override
  public void periodic() {
    // should this be getTurretDegrees????
    SmartDashboard.putNumber("Turret position degrees", getTurretPosition());

    // Report reaching limits
    TurretRangeState turretRangeState = getTurretRange();
    if (turretRangeState == TurretRangeState.OVER) {
      SmartDashboard.putString("Turret position at Maximum Limit"," ");
    } else if (turretRangeState == TurretRangeState.UNDER) {
      SmartDashboard.putString("Turret position at Minimum Limit"," ");
    }
  }

  public void setPower(double power){
    m_turretMotor.set(ControlMode.PercentOutput, power);
  }

  public void setPosition(double degrees){
    m_turretMotor.set(ControlMode.Position, degreesToSRX(degrees));
  }

  public void stopMotor(){
    setPower(0);
  }


  public double getTurretPosition(){
    return m_turretMotor.getSelectedSensorPosition() * ConversionConstants.kTurretTicksPerRotation;
  }

  public TurretRangeState getTurretRange(){
     double degrees = getTurretDegrees();
     if (degrees > maxWorkingLimit) {
       return TurretRangeState.OVER;
     }
     else if (degrees < minWorkingLimit) {
      return TurretRangeState.UNDER;
     }
     else return TurretRangeState.NORMAL;
  }

  public void correctTurretRange(){
    TurretRangeState turretRangeState = getTurretRange();
    if (turretRangeState == TurretRangeState.OVER) {
      SmartDashboard.putString("Correcting Turret position from Maximum Limit"," ");
      setPosition( getTurretDegrees() - 360);
    } else if (turretRangeState == TurretRangeState.UNDER) {
      SmartDashboard.putString("Correcting Turret position from Minimum Limit"," ");
      setPosition( getTurretDegrees() + 360);
    }
  }

  public double getTurretDegrees(){
    return srxToDegrees(m_turretMotor.getSelectedSensorPosition());
  }

  /** 
  * Returns the angle of the turret relative to the field.
  * 0 degrees is facing opponent's alliance stations.
  */
  public double getTurretFieldDegrees(){
    return 420; //Placeholder
  }

  //Grabs the PIDF values from Smartdashboard/Shuffboard
  public void configTurretFeedbackGains(){
    kP = SmartDashboard.getNumber("Turret kP", kP);
    kF = SmartDashboard.getNumber("Turret kF", kF);

    m_turretMotor.config_kP(0, kP);
    m_turretMotor.config_kI(0, 0);
    m_turretMotor.config_IntegralZone(0, 0);
    m_turretMotor.config_kD(0, 0);
    m_turretMotor.config_kF(0, kF);

    System.out.println("Turret gains configed: kP " + kP + "kF " + kF);
  }

  //May be easier to move to command for logic
  public void searchForTarget(){

  }

  public void setTurretState(TurretState state){
    m_turretState = state;

    switch(state){
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

  public TurretState getTurretState(){
    return m_turretState;
  }

  //Used to convert native encoder units to turret degrees
  private double degreesToSRX(double degrees){
    return degrees / (ConversionConstants.kTurretDegreesPerRotation * ConversionConstants.kTurretTicksPerRotation);
  }

  private double srxToDegrees(double srx){
    return srx * ConversionConstants.kTurretTicksPerRotation * ConversionConstants.kTurretDegreesPerRotation;
  }
}
