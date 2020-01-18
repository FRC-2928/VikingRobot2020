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

  //Feedback gains
  private double kP = 0;
  private double kF = 0;

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

    m_turretMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 60, 20));

    m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    //Used to config PIDF gains
    SmartDashboard.putNumber("Turret kP", kP);
    SmartDashboard.putNumber("Turret kF", kF);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret position degrees", getTurretPosition());
  }

  public void setPower(double power){
    m_turretMotor.set(ControlMode.PercentOutput, power);
  }

  public void setPosition(double degrees){
    m_turretMotor.set(ControlMode.Position, degreesToSRX(degrees));
  }

  public double getTurretPosition(){
    return m_turretMotor.getSelectedSensorPosition() * ConversionConstants.kTurretTicksPerRotation;
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

  public void searchForTarget(){
    
  }

  //Used to convert native encoder units to turret degrees
  private double degreesToSRX(double degrees){
    return degrees / (ConversionConstants.kTurretDegreesPerRotation * ConversionConstants.kTurretTicksPerRotation);
  }

  private double srxToDegrees(double srx){
    return srx * ConversionConstants.kTurretTicksPerRotation * ConversionConstants.kTurretDegreesPerRotation;
  }
}
