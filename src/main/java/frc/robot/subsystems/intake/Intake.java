/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;
// import frc.robot.Robot;
import frc.robot.Constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;



public class Intake extends SubsystemBase {
  /**
   * Creates a new intake.
   */

  private Solenoid kIntakeSolenoidRightExtend;
  private Solenoid kIntakeSolenoidRightRetract;
  private Solenoid kIntakeSolenoidLeftExtend;
  private Solenoid kIntakeSolenoidLeftRetract;

  private WPI_TalonSRX m_intakeMotor;

  private boolean isExtended;
  

   public Intake() {
    kIntakeSolenoidRightExtend = new Solenoid(RobotMap.kIntakeSoleniodRightOne);
    kIntakeSolenoidRightRetract = new Solenoid(RobotMap.kIntakeSoleniodRightTwo);
    kIntakeSolenoidLeftExtend= new Solenoid(RobotMap.kIntakeSoleniodLeftOne);
    kIntakeSolenoidLeftRetract= new Solenoid(RobotMap.kIntakeSoleniodLeftTwo);
  
    m_intakeMotor = new WPI_TalonSRX(RobotMap.kIntakeWPI_TalonSRX);

    m_intakeMotor.configFactoryDefault();

    m_intakeMotor.configVoltageCompSaturation(12);
    m_intakeMotor.enableVoltageCompensation(true);
    m_intakeMotor.configNominalOutputForward(0);
    m_intakeMotor.configNominalOutputReverse(0);
    m_intakeMotor.configNeutralDeadband(0.01);
    m_intakeMotor.setNeutralMode(NeutralMode.Coast);
 
    m_intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 35, 0.04));
 
    // m_intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public void extendIntake () {
    kIntakeSolenoidRightExtend.set(true);
    kIntakeSolenoidLeftExtend.set(true);
    kIntakeSolenoidLeftRetract.set(false);
    kIntakeSolenoidRightRetract.set(false);

    isExtended = true;

  }

  public void retractIntake () {

    kIntakeSolenoidRightExtend.set(false);
    kIntakeSolenoidLeftExtend.set(false);
    kIntakeSolenoidLeftRetract.set(true);
    kIntakeSolenoidRightRetract.set(true);

    isExtended = false;
  }

  public void setPower (double power) {
    m_intakeMotor.set(ControlMode.PercentOutput, power);
  }

  public void startMotor () {
    setPower(0.5);
  }

  public void stopMotor () {
    setPower(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double current = m_intakeMotor.getSupplyCurrent(); 
    SmartDashboard.putNumber("intake current", current);

  }
}
