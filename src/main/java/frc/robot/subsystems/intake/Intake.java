/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotMap;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class Intake extends SubsystemBase {
  /**
   * Creates a new intake.
   */

  private Solenoid kIntakeSolenoidRightBase;
  private Solenoid kIntakeSolenoidRightArm;
  private Solenoid kIntakeSolenoidLeftBase;
  private Solenoid kIntakeSolenoidLeftArm;

  private WPI_TalonSRX m_intakeMotor;

  public enum IntakeState {
    GROUND_PICKUP,STATION_PICKUP,STOWED;
  }

  private IntakeState currentState;

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public Intake() {
    kIntakeSolenoidRightBase = new Solenoid(RobotMap.kIntakeSoleniodRightOne);
    kIntakeSolenoidRightArm = new Solenoid(RobotMap.kIntakeSoleniodRightTwo);
    kIntakeSolenoidLeftBase= new Solenoid(RobotMap.kIntakeSoleniodLeftOne);
    kIntakeSolenoidLeftArm= new Solenoid(RobotMap.kIntakeSoleniodLeftTwo);

    m_intakeMotor = new WPI_TalonSRX(RobotMap.kIntakeWPI_TalonSRX);

    m_intakeMotor.configFactoryDefault();

    m_intakeMotor.configVoltageCompSaturation(12);
    m_intakeMotor.enableVoltageCompensation(true);
    m_intakeMotor.configNominalOutputForward(0);
    m_intakeMotor.configNominalOutputReverse(0);
    m_intakeMotor.configNeutralDeadband(0.01);
    m_intakeMotor.setNeutralMode(NeutralMode.Coast);
    m_intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 35, 0.04));

    // Set default command
    setDefaultCommand(new RunCommand(this::stowIntake, this)); 
  }

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double current = m_intakeMotor.getSupplyCurrent(); 
    SmartDashboard.putNumber("intake current", current);

  }

  public void groundPickup () {
    moveIntake(IntakeState.GROUND_PICKUP);
  }

  public void stationPickup () {
    moveIntake(IntakeState.STATION_PICKUP);
  }

  // This is the default command
  public void stowIntake() {
    stopMotor();
    moveIntake(IntakeState.STOWED);
  }

  public void moveIntake(IntakeState state) {

    switch (state) {
      case GROUND_PICKUP: 
        kIntakeSolenoidLeftArm.set(false);
        kIntakeSolenoidRightArm.set(false);
        kIntakeSolenoidLeftBase.set(true);
        kIntakeSolenoidRightBase.set(true);
      break;

      case STATION_PICKUP: 
        kIntakeSolenoidLeftArm.set(true);
        kIntakeSolenoidRightArm.set(true);
        kIntakeSolenoidLeftBase.set(true);
        kIntakeSolenoidRightBase.set(true);
      break;

      case STOWED:
        kIntakeSolenoidLeftArm.set(true);
        kIntakeSolenoidRightArm.set(true);
        kIntakeSolenoidLeftBase.set(false);
        kIntakeSolenoidRightBase.set(false);
      break;

      default:
      break;
      }

      currentState = state;
    }

  // -----------------------------------------------------------
  // Actuator Output
  // -----------------------------------------------------------  
  public void setPower (double power) {
    m_intakeMotor.set(ControlMode.PercentOutput, power);
  }

  public void startMotor () {
    setPower(0.5);
  }

  public void stopMotor () {
    setPower(0);
  }

}
