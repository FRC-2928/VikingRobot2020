package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

/**
 * Feedersubsystem is responsible for feeding balls into the shooter
 * We'll use sensors to keep track of ball positions to hold some in the tower
 * We can't have too many in the hopper or they'll jam
*/

public class FeederSubsystem extends SubsystemBase {
  private WPI_VictorSPX m_hopperMotor;
  private WPI_VictorSPX m_towerMotor;

  //IR sensors to detect ball positions
  private DigitalOutput m_bottomSensor;
  private DigitalOutput m_middleSensor;
  private DigitalOutput m_topSensor;

  private FeederState m_feederState;
  private IndexState m_indexState;

  public enum FeederState{
    STOPPED, WAITING, INDEXING, FEEDING, FULL;
  }

  public enum IndexState{
    NONE, ONE_BALL, TWO_BALLS, THREE_BALLS;
  }

  public FeederSubsystem() {
   m_hopperMotor = new WPI_VictorSPX(RobotMap.kHopperVictorSPX);
   m_towerMotor = new WPI_VictorSPX(RobotMap.kTowerVictorSPX);

   m_bottomSensor = new DigitalOutput(RobotMap.kIRSensorBottom);
   m_middleSensor = new DigitalOutput(RobotMap.kIRSensorMiddle);
   m_topSensor = new DigitalOutput(RobotMap.kIRSensorTop);

   for(WPI_VictorSPX feederMotors: new WPI_VictorSPX[]{m_hopperMotor, m_towerMotor}){
   feederMotors.configFactoryDefault();

   feederMotors.configVoltageCompSaturation(12);
   feederMotors.enableVoltageCompensation(true);
   feederMotors.configNominalOutputForward(0);
   feederMotors.configNominalOutputReverse(0);
   feederMotors.configNeutralDeadband(0.01);

   feederMotors.setNeutralMode(NeutralMode.Coast);
   }
  }

  public void setFeederState(FeederState state){
    switch(state){
      case STOPPED:
      break;

      case WAITING:
      break;

      case INDEXING:
      break;

      case FEEDING:
      break;

      case FULL:
      break;
    }
  }

  public void setHopperPower(double power){
    m_hopperMotor.set(ControlMode.PercentOutput, power);
  }

  public void setTowerPower(double power){
    m_towerMotor.set(ControlMode.PercentOutput, power);
  } 

  public void index(){    
    
  }

  // public IndexState getIndexState(){
    
  // }

  public boolean getBottomSensor(){
    return m_bottomSensor.get();
  }

  public boolean getMiddleSensor(){
    return m_middleSensor.get();
  }

  public boolean getTopSensor(){
    return m_topSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}