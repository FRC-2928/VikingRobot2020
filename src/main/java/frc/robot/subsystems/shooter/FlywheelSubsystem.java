package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  }

  public double getFlywheelVelocity(){
    return m_flywheelMotor.getSelectedSensorVelocity();
  }

  // public void setFlywheelPID(){
  //   m_flywheelMotor.config_kP(slotIdx, value)
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
