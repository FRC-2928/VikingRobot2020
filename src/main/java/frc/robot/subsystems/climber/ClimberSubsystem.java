package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

/**
 * ClimberSubsystem encompasses the elevator lift and the gearbox brake.
 * It's deployed to 3 different setpoints and then brought down to climb
 */
public class ClimberSubsystem extends SubsystemBase {
  private WPI_TalonFX m_climberMotor;
  private Solenoid m_climberBrake;

  private BrakeState currentBrakeState;
  private ClimberState currentClimberState;

  public enum ClimberState{
    STOWED, DEPLOYING, DEPLOYED, CLIMBED;
  }

  public enum BrakeState{
    OFF, ON;
  }

  public ClimberSubsystem() {
    m_climberMotor = new WPI_TalonFX(RobotMap.kClimberTalonFX);
    m_climberBrake = new Solenoid(RobotMap.kClimberSolenoidBrake);

    m_climberMotor.configFactoryDefault();

    m_climberMotor.configVoltageCompSaturation(12);
    m_climberMotor.enableVoltageCompensation(true);
    m_climberMotor.configNominalOutputForward(0);
    m_climberMotor.configNominalOutputReverse(0);
    m_climberMotor.configNeutralDeadband(0.01);
    m_climberMotor.setNeutralMode(NeutralMode.Brake);

    m_climberMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 0.04));

    m_climberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setElevatorPower(double power){
    m_climberMotor.set(ControlMode.PercentOutput, power);
  }

  public double getElevatorEncoder(){
    return m_climberMotor.getSelectedSensorPosition();
  }

  // //
  // public double getElevatorPosition(){}

  public void setBrakeState(BrakeState state){
    currentBrakeState = state;

    switch(state){
      case ON:
      setSolenoid(true);
      break;

      case OFF:
      setSolenoid(false);
      break;
    }
  }

  public void setSolenoid(boolean state){
    if(state == true){
      m_climberBrake.set(true);
    }

    if(state == false){
      m_climberBrake.set(false);
    }
  }


}
