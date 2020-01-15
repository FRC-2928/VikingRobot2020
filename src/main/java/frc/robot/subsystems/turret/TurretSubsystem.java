package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.RobotMap;
/**
* TurretSubsystem is responsible for subsystem level logic with the turret.
*/
public class TurretSubsystem extends SubsystemBase {
  private WPI_TalonSRX m_turretMotor;

  public TurretSubsystem() {
    m_turretMotor = new WPI_TalonSRX(RobotMap.kTurretTalonSRX);

    m_turretMotor.configFactoryDefault();

    m_turretMotor.configVoltageCompSaturation(12);
    m_turretMotor.enableVoltageCompensation(true);
    m_turretMotor.configNominalOutputForward(0);
    m_turretMotor.configNominalOutputReverse(0);
    m_turretMotor.configNeutralDeadband(0.01);
    m_turretMotor.setNeutralMode(NeutralMode.Brake);

    m_turretMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 20));

    // m_turretMotor.configSelectedFeedbackSensor(feedbackDevice.)

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power){
    m_turretMotor.set(ControlMode.PercentOutput, power);
  }

  public double getTurretPosition(){
    return m_turretMotor.getSelectedSensorPosition() * ConversionConstants.kTurretTicksPerRotation;
  }

  public double getTurretDegrees(){
    return getTurretPosition() * ConversionConstants.kTurretDegreesPerRotation;
  }

  /** 
  * Returns the angle of the turret relative to the field.
  * 0 degrees is facing opponent's alliance stations.
  */
  public double getTurretFieldDegrees(){
    return 420; //Placeholder
  }
}
