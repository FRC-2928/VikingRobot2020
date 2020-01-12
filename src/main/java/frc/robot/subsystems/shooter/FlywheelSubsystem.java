package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Conversions;
import frc.robot.Constants.RobotMap;
/**
   * Test FlywheelSubsystem to handle shooting and velocity controls
   */
public class FlywheelSubsystem extends SubsystemBase {
  private TalonFX m_flywheelMotor;
  private double velocity;

  public FlywheelSubsystem() {
   m_flywheelMotor = new TalonFX(RobotMap.kFlywheelTalonFX);

   m_flywheelMotor.configFactoryDefault();

   m_flywheelMotor.configVoltageCompSaturation(12);
   m_flywheelMotor.enableVoltageCompensation(true);
   m_flywheelMotor.configNominalOutputForward(0);
   m_flywheelMotor.configNominalOutputReverse(0);
   m_flywheelMotor.configNeutralDeadband(0.01);
   m_flywheelMotor.setNeutralMode(NeutralMode.Coast);

   m_flywheelMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 0.04));

   m_flywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
   m_flywheelMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
   m_flywheelMotor.configVelocityMeasurementWindow(64);
   configFeedbackGains();

   setDefaultCommand(new RunCommand(this::stopFlywheel, this));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel RPM", getFlywheelVelocityRPM());
  }

  public void setPower(double power){
    m_flywheelMotor.set(ControlMode.PercentOutput, power);
  }

  public double getFlywheelVelocityRPM(){
    return fxToRPM(m_flywheelMotor.getSelectedSensorVelocity());
  }

  public void setFlywheelRPM(double rpm){
    m_flywheelMotor.set(ControlMode.Velocity, rpmToFX(rpm));
  }

  public void stopFlywheel(){
    setPower(0);
  }  

  public void configFeedbackGains(){
    double kP = SmartDashboard.getNumber("Flywheel kP", 0);
    double kF = SmartDashboard.getNumber("Flywheel kF", 0);

    m_flywheelMotor.config_kP(0, kP);
    m_flywheelMotor.config_kI(0, SmartDashboard.getNumber("Flywheel kI", 0));
    m_flywheelMotor.config_IntegralZone(0, (int)SmartDashboard.getNumber("Flywheel IntegralZone", 0));
    m_flywheelMotor.config_kD(0, SmartDashboard.getNumber("Flywheel kD", 0));
    m_flywheelMotor.config_kF(0, kF);

    System.out.println("Flywheel configed yay: kP: " + kP + " kF: " + kF);
  }

  private double rpmToFX(double rpm){
    return rpm*Conversions.kFlywheelEncoderTicksPerRotation/600;
  }

  private double fxToRPM(double fx){
    return fx/Conversions.kFlywheelEncoderTicksPerRotation * 600;
  }
}