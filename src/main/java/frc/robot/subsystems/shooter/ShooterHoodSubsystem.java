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
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;
/**
   * Creates a new ShooterHoodSubsystem.
   */
public class ShooterHoodSubsystem extends SubsystemBase {
  private WPI_TalonSRX m_hoodMotor;

  private final double kP = PIDConstants.kHoodkP;
  private final double kD = PIDConstants.kHoodkD;
  
  public ShooterHoodSubsystem() {
    m_hoodMotor = new WPI_TalonSRX(RobotMap.kHoodTalonSRX);

    m_hoodMotor.configFactoryDefault();

    //6 volts is 100% enough
    m_hoodMotor.configVoltageCompSaturation(6);
    m_hoodMotor.enableVoltageCompensation(true);
    
    //Minimum output is 0.1 so the PD loop can do small adjustments
    m_hoodMotor.configNominalOutputForward(0.1);
    m_hoodMotor.configNominalOutputReverse(0.1);

    //No need to have a deadband since we have a nominal output
    m_hoodMotor.configNeutralDeadband(0);

    m_hoodMotor.setNeutralMode(NeutralMode.Brake);

    //Invert the sensors
    m_hoodMotor.setSensorPhase(true);

    //Hood literally draws 12 amps at max but better safe than sorry
    m_hoodMotor.configPeakCurrentLimit(40);
    m_hoodMotor.enableCurrentLimit(true);
    m_hoodMotor.configPeakCurrentDuration(60);
    m_hoodMotor.configContinuousCurrentLimit(30);
    
    m_hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    
    m_hoodMotor.configAllowableClosedloopError(0, 5);

    resetHoodEncoder();

    // setDefaultCommand(new RunCommand(this::stopHood, this));

    //Placing the hood gains on ShuffleBoard
    SmartDashboard.putNumber("Hood kP", kP);
    SmartDashboard.putNumber("Hood kD", kD);
    SmartDashboard.putNumber("Hood kF", 0);
    SmartDashboard.putNumber("Hood Target Degrees", 0);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood native units", m_hoodMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Hood Position", getHoodDegrees());
    SmartDashboard.putNumber("Hood Voltage Draw", m_hoodMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("Hood Amp Draw", m_hoodMotor.getSupplyCurrent());
  }

  public double getHoodRotation(){
    return m_hoodMotor.getSelectedSensorPosition() / ConversionConstants.kHoodEncoderTicksPerRotation / ConversionConstants.kHoodGearRatio;
  }

  public double getHoodDegrees(){
    return getHoodRotation() * 360;
  }

  //For tuning gains, will take out once we've finalized everything
  public void configPIDGains(){
    double newkP = SmartDashboard.getNumber("Hood kP", kP);
    double newkD = SmartDashboard.getNumber("Hood kD", kD);
    double newkF = SmartDashboard.getNumber("Hood kF", 0);

    m_hoodMotor.config_kP(0, newkP);
    m_hoodMotor.config_kI(0, SmartDashboard.getNumber("Hood kI", 0));
    m_hoodMotor.config_IntegralZone(0, (int)SmartDashboard.getNumber("Hood IntegralZone", 0));
    m_hoodMotor.config_kD(0, newkD);
    m_hoodMotor.config_kF(0, newkF);

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
