package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

/**
 * Feedersubsystem is responsible for feeding balls into the shoote
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

  private final double m_indexPowerInitial = FeederConstants.indexPower;
  private final double m_indexSetpointInitial = FeederConstants.indexSetpoint;
  private double m_indexPower = m_indexPowerInitial;
  private double m_indexSetpoint = m_indexSetpointInitial;

  public enum FeederState{
    STOPPED, WAITING, INDEXING, FEEDING, FULL;
  }

  public enum IndexState{
    WAITING_TO_INDEX, READY_TO_INDEX, FULL;
  }

  private FeederState m_feederState = FeederState.WAITING;
  private IndexState m_indexState = IndexState.WAITING_TO_INDEX;

  // ---- Constructor -----
  public FeederSubsystem() {

    m_hopperMotor = new WPI_VictorSPX(FeederConstants.kHopperVictorSPX);
    m_towerMotor = new WPI_VictorSPX(FeederConstants.kTowerVictorSPX);

    m_bottomSensor = new DigitalOutput(FeederConstants.kIRSensorBottom);
    m_middleSensor = new DigitalOutput(FeederConstants.kIRSensorMiddle);
    m_topSensor = new DigitalOutput(FeederConstants.kIRSensorTop);

    for(WPI_VictorSPX feederMotors: new WPI_VictorSPX[]{m_hopperMotor, m_towerMotor}){
      feederMotors.configFactoryDefault();
      feederMotors.configVoltageCompSaturation(12);
      feederMotors.enableVoltageCompensation(true);
      feederMotors.configNominalOutputForward(0);
      feederMotors.configNominalOutputReverse(0);
      feederMotors.configNeutralDeadband(0.01);
      feederMotors.setNeutralMode(NeutralMode.Coast);
    }
    
    m_towerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_towerMotor.configAllowableClosedloopError(0, 5);

    resetIndexEncoder();

    m_indexState = IndexState.WAITING_TO_INDEX;

    //Placing the indexing values on ShuffleBoard
    SmartDashboard.putNumber("Index Power", m_indexPowerInitial);
    SmartDashboard.putNumber("Index Setpoint", m_indexSetpointInitial);
    SmartDashboard.putString("Index State", m_indexState.name());
   
  }

  @Override
  public void periodic() {
    // Moves incoming ball up the tower. 
    // This method will be called once per scheduler
    index();
  }

  // Set power to the hopper motor
  public void setHopperPower(double power){
    m_hopperMotor.set(ControlMode.PercentOutput, power);
  }

  // Set power to the tower motor
  public void setIndexPower(double power){
    m_towerMotor.set(ControlMode.PercentOutput, power);
  } 

  // Stop the hopper
  public void startHopper() {
    setHopperPower(0.4);
  }
  // Stop the hopper
  public void stopHopper() {
    setHopperPower(0);
  }

  /**
   * Moves the ball up the tower.  There are two options for doing this
   * so we can test which one works best.
   * Comment out the one that you are not going to use.  
  */
  public void startIndex() {

    // Move until index sensor is tripped. This happens in checkIndexState()
    setIndexPower(m_indexPower);

    // Run a PID loop within the Talon controller.
    m_towerMotor.set(ControlMode.Position, m_indexSetpoint);

  }

  public void stopIndex() {
    setIndexPower(0);
  }

  //Called in a loop at periodic
  //Handles the indexing of balls in the tower
  public void index(){

    // Check if we should take any action on the index
    checkIndexState();

    if (m_indexState == IndexState.FULL) {stopIndex();}
    // TBD: if here are a buch of balls in the hopper we may have to stop the hopper?

    if (m_indexState == IndexState.READY_TO_INDEX) {
      stopHopper(); // Prevent more balls from coming in
      startIndex(); // Move the ball up the tower
      m_feederState = FeederState.INDEXING;
    } else {
      stopIndex(); // Done moving ball up the tower    
    }

    if (m_indexState == IndexState.WAITING_TO_INDEX) {

      stopIndex(); // full and don't want to add a ball.
      startHopper(); // Get more balls in
    }

  }

  /** 
   * Check the state of the index to determine what action should 
   * be taken next.
  */
  public void checkIndexState(){

    // There's a ball at the top so don't index
    if(topSensorTripped()){
      m_indexState = IndexState.FULL;
      return;
    }

    if(bottomSensorTripped() && !topSensorTripped()){
      // We have a ball on the bottom and the top slot is open
      m_indexState = IndexState.READY_TO_INDEX;
    } else {
      // Waiting for a new ball to come in and top slot is open
      m_indexState = IndexState.WAITING_TO_INDEX;
    }
  
  }

  public void setIndexState(IndexState state) {
    m_indexState = state;
  }

  public boolean bottomSensorTripped(){
    return m_bottomSensor.get();
  }

  // public boolean middleSensorTripped(){
  //   return m_middleSensor.get();
  // }

  public boolean topSensorTripped(){
    return m_topSensor.get();
  }

  public void resetIndexEncoder(){
    m_towerMotor.setSelectedSensorPosition(0);
  }

  // These are for calibrating the index 
  public void configIndexPower() {
    m_indexPower = SmartDashboard.getNumber("Index Power", m_indexPowerInitial);
  }

  public void configIndexSetpoint() {
    m_indexSetpoint = SmartDashboard.getNumber("Index Setpoint", m_indexSetpointInitial);
  }

}