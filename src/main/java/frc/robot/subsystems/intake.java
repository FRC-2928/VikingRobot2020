/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;



public class intake extends SubsystemBase {
  /**
   * Creates a new intake.
   */

  private Solenoid kIntakeSolenoidRightExtend;
  private Solenoid kIntakeSolenoidRightRetract;
  private Solenoid kIntakeSolenoidLeftExtend;
  private Solenoid kIntakeSolenoidLeftRetract;

  private TalonFX m_intakeMotor;
  

   public intake() {
    kIntakeSolenoidRightExtend = new Solenoid(RobotMap.kIntakeSoleniodOne);
    kIntakeSolenoidRightRetract = new Solenoid(RobotMap.kIntakeSoleniodTwo);
    kIntakeSolenoidLeftExtend= new Solenoid(RobotMap.kIntakeSoleniodThree);
    kIntakeSolenoidLeftRetract= new Solenoid(RobotMap.kIntakeSoleniodFour);
  
    m_intakeMotor = new TalonFX(RobotMap.kIntakeTalonFX);

    m_intakeMotor.configFactoryDefault();

    m_intakeMotor.configVoltageCompSaturation(12);
    m_intakeMotor.enableVoltageCompensation(true);
    m_intakeMotor.configNominalOutputForward(0);
    m_intakeMotor.configNominalOutputReverse(0);
    m_intakeMotor.configNeutralDeadband(0.01);
    m_intakeMotor.setNeutralMode(NeutralMode.Coast);
 
    m_intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 0.04));
 
    m_intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public void ExtendIntake () {
    kIntakeSolenoidLeftRetract.set(false);
    kIntakeSolenoidRightRetract.set(false);
    kIntakeSolenoidRightExtend.set(true);
    kIntakeSolenoidLeftExtend.set(true);
    
  }

  public void RetractIntake () {
    kIntakeSolenoidRightExtend.set(false);
    kIntakeSolenoidLeftExtend.set(false);
    kIntakeSolenoidLeftRetract.set(true);
    kIntakeSolenoidRightRetract.set(true);
  }

  public void CloseIntake () {
    kIntakeSolenoidLeftRetract.close();
    kIntakeSolenoidLeftExtend.close();
    kIntakeSolenoidRightRetract.close();
    kIntakeSolenoidRightExtend.close();
  }

  public void SetPower (double power) {
    m_intakeMotor.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double current = m_intakeMotor.getSupplyCurrent(); 
    SmartDashboard.putNumber("intake current", current);
  }
}
