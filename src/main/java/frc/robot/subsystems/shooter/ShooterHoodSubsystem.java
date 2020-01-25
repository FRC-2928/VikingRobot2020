package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.RobotMap;
/**
   * Creates a new ShooterHoodSubsystem.
   */
public class ShooterHoodSubsystem extends SubsystemBase {
  private WPI_TalonSRX m_hoodMotor;
  
  public ShooterHoodSubsystem() {
    m_hoodMotor = new WPI_TalonSRX(RobotMap.kHoodTalonSRX);

    m_hoodMotor.configFactoryDefault();

    m_hoodMotor.configVoltageCompSaturation(6);
    m_hoodMotor.enableVoltageCompensation(true);
    m_hoodMotor.configNominalOutputForward(0);
    m_hoodMotor.configNominalOutputReverse(0);
    m_hoodMotor.configNeutralDeadband(0.01);
    m_hoodMotor.setNeutralMode(NeutralMode.Brake);

    m_hoodMotor.setSensorPhase(true);
    m_hoodMotor.setInverted(InvertType.InvertMotorOutput);

    m_hoodMotor.configPeakCurrentLimit(45);
    m_hoodMotor.enableCurrentLimit(true);
    m_hoodMotor.configContinuousCurrentLimit(30);
    
    m_hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    SmartDashboard.putNumber("Hood encoder values", getHoodRotation());
    m_hoodMotor.setSelectedSensorPosition(0);

  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Rotation", getHoodRotation());
    SmartDashboard.putNumber("Hood native units", m_hoodMotor.getSelectedSensorPosition());
  }

  public double getHoodRotation(){
    return m_hoodMotor.getSelectedSensorPosition() / ConversionConstants.kHoodEncoderTicksPerRotation / ConversionConstants.kHoodGearRatio;
  }

  public void setPower(double power){
    m_hoodMotor.set(ControlMode.PercentOutput, power);
  }

}