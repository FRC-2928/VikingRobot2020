package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    m_hoodMotor.configNominalOutputForward(0.1);
    m_hoodMotor.configNominalOutputReverse(0.1);
    m_hoodMotor.configNeutralDeadband(0.01);
    m_hoodMotor.setNeutralMode(NeutralMode.Brake);

    m_hoodMotor.setSensorPhase(true);

    m_hoodMotor.configPeakCurrentLimit(45);
    m_hoodMotor.enableCurrentLimit(true);
    m_hoodMotor.configContinuousCurrentLimit(30);
    
    m_hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    
    resetHoodEncoder();

    // setDefaultCommand(new RunCommand(this::stopHood, this));

    SmartDashboard.putNumber("Hood kP", 2.5);
    SmartDashboard.putNumber("Hood kD", 15);
    SmartDashboard.putNumber("Hood kF", 0);
    SmartDashboard.putNumber("Hood Target Degrees", 0);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood native units", m_hoodMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Hood Position", getHoodDegrees());
    SmartDashboard.putNumber("Hood voltage draw", m_hoodMotor.getMotorOutputVoltage());
  }

  public double getHoodRotation(){
    return m_hoodMotor.getSelectedSensorPosition() / ConversionConstants.kHoodEncoderTicksPerRotation / ConversionConstants.kHoodGearRatio;
  }

  public double getHoodDegrees(){
    return getHoodRotation() * 360;
  }

  public void configPIDGains(){
    double kP = SmartDashboard.getNumber("Hood kP", 0);
    double kD = SmartDashboard.getNumber("Hood kD", 0);
    double kF = SmartDashboard.getNumber("Hood kF", 0);

    m_hoodMotor.configAllowableClosedloopError(0, 5);

    m_hoodMotor.config_kP(0, kP);
    m_hoodMotor.config_kI(0, SmartDashboard.getNumber("Hood kI", 0));
    m_hoodMotor.config_IntegralZone(0, (int)SmartDashboard.getNumber("Hood IntegralZone", 0));
    m_hoodMotor.config_kD(0, kD);
    m_hoodMotor.config_kF(0, kF);

    System.out.println("Hood configed");
  }

  public void setHoodDegrees(){
    double target = degreesToSRX(SmartDashboard.getNumber("Hood Target Degrees", 0));
    SmartDashboard.putNumber("Actual Hood Target", target);
    m_hoodMotor.set(ControlMode.Position, target);
  }

  public void setPower(double power){
    m_hoodMotor.set(ControlMode.PercentOutput, power);
  }

  public void stopHood(){
    m_hoodMotor.set(ControlMode.PercentOutput, 0);
  }

  public void resetHoodEncoder(){
    m_hoodMotor.setSelectedSensorPosition(0);
  }

  private double srxToDegrees(double srx){
    return srx * 360 / ConversionConstants.kHoodEncoderTicksPerRotation / ConversionConstants.kHoodGearRatio;
  }

  private double degreesToSRX(double degrees){
    return degrees / 360 * ConversionConstants.kHoodEncoderTicksPerRotation * ConversionConstants.kHoodGearRatio;
  }
}
