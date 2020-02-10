package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;;

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

  private final double m_indexPowerInitial = PIDConstants.indexPower;
  private final double m_indexSetpointInitial = PIDConstants.indexSetpoint;
  private double m_indexPower = m_indexPowerInitial;

  public enum HopperState{
    STOPPED, FEEDING, REVERSED, HALTED;
  }

  public enum IndexState{
    WAITING_TO_INDEX, READY_TO_INDEX, INDEXING, FULL_BUT_RECEIVING, FULL, HALTED, REVERSED;
  }

  private HopperState m_hopperState = HopperState.STOPPED;
  private IndexState m_indexState = IndexState.WAITING_TO_INDEX;

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
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
    
    m_towerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_towerMotor.configAllowableClosedloopError(0, 5);

    resetIndexEncoder();

    // Set default command
    setDefaultCommand(new RunCommand(this::stopFeeder, this));    

    //Placing the indexing values on ShuffleBoard
    SmartDashboard.putNumber("Index Power", m_indexPowerInitial);
    SmartDashboard.putNumber("Index Setpoint", m_indexSetpointInitial);
    SmartDashboard.putString("Index State", m_indexState.name());
   
  }

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    // Moves incoming ball up the tower. 
    // This method will be called once per scheduler
    // runFeeder();
  }

  public void stopFeeder() {
    stopIndex();
    setIndexState(IndexState.HALTED);
    stopHopper();
    setHopperState(HopperState.HALTED);
  }

  
  // Check the state of the index to determine what action should be taken next.
  public void runFeeder() {

    // The bottom sensor will remain tripped until the ball clears the
    // sensor, so stay in the INDEXING state.
    if (m_indexState == IndexState.INDEXING && bottomSensorTripped()) {
      indexingAction(); // no action taken
      return;
    }

    // INDEXING with bottom sensor now cleared.
    if (m_indexState == IndexState.INDEXING) {
      if (topSensorTripped()) {
        m_indexState = IndexState.FULL_BUT_RECEIVING; // States 5 & 6
        fullButReceivingAction(); // stop indexer, start hopper
      } else if (middleSensorTripped()) {
        m_indexState = IndexState.WAITING_TO_INDEX;
        waitingToIndexAction(); // stop indexer, start hopper
      }
      return;
    }

    // Not currently INDEXING

    // Got a new ball
    if (bottomSensorTripped()) {
      if (topSensorTripped()) {
        m_indexState = IndexState.FULL;  // states 7 & 8
        fullAction(); // stop hopper and indexer
      } 
      else {
        m_indexState = IndexState.READY_TO_INDEX; // states 3 & 4
        readyToIndexAction(); // stop hopper, start indexer
        m_indexState = IndexState.INDEXING;
      }
    } 
    // The index is completely empty
    if (allSensorsCleared()) {
      m_indexState = IndexState.WAITING_TO_INDEX;
      waitingToIndexAction(); // start hopper
    }
  }

  // ----- Actions taken depending on the state -----

  public void fullButReceivingAction() {
    stopIndex();     
    startHopper(); // The bottom slot is empty so start the hopper
  }

  public void waitingToIndexAction() {
    stopIndex(); // Index should be stopped
    startHopper(); // Get more balls in
  }

  public void readyToIndexAction() {
    stopHopper(); // Prevent more balls from coming in

    // Move until index sensor is tripped. This happens in runFeeder()
    setIndexPower(m_indexPower);
  }

  public void fullAction() {
    stopIndex();
    stopHopper();
  }

  public void indexingAction() {
    // Nothing to do
  }

  // ---------- End of actions  ---------------

  // Use this to reverse all balls out of the feeder
  public void reverseFeeder() {
    reverseHopper();
    WaitCommand waitCommand = new WaitCommand(0.5);
    waitCommand.execute();
    reverseIndex();
  }

  // Used to feed balls while shooting
  public void fastForwardFeeder() {
    setIndexPower(Constants.FeederConstants.kFastForwardPower);
    setHopperPower(Constants.FeederConstants.kFastForwardPower);
  }

  // Public assess to the IndexState
  public void setIndexState(IndexState state) {
    m_indexState = state;
  }

  // Public assess to the HopperState
  public void setHopperState(HopperState state) {
    m_hopperState = state;
  }

  // -----------------------------------------------------------
  // Actuator Output
  // -----------------------------------------------------------

  // Set power to the hopper motor
  public void setHopperPower(double power){
    m_hopperMotor.set(ControlMode.PercentOutput, power);
  }

  // Set power to the tower motor
  public void setIndexPower(double power){
    m_towerMotor.set(ControlMode.PercentOutput, power);
  } 

  // Start the hopper
  public void startHopper() {
    m_hopperState = HopperState.FEEDING;
    setHopperPower(0.4);
  }

  // Stop the hopper
  public void stopHopper() {
    setHopperPower(0);
    m_hopperState = HopperState.STOPPED;
  }

  // Reverse the hopper
  public void reverseHopper() {
    m_hopperState = HopperState.REVERSED;
    setHopperPower(-0.4);
  }

  // Reverse the indexer
  public void reverseIndex() {
    m_indexState = IndexState.REVERSED;
    setIndexPower(-0.4);
  }

  // Stop the indexer
  public void stopIndex() {
    setIndexPower(0);
  }

  // Toggle the hopper on and off while in FEEDING state.
  public void toggleFeedingHopper() {
    if (m_hopperState == HopperState.FEEDING) {
      stopHopper();
    } else {
      startHopper();
    }  
  }

  // Toggle the hopper on and off while in REVERSED state.
  public void toggleReversedHopper() {
    if (m_hopperState == HopperState.REVERSED) {
      stopHopper();
    } else {
      startHopper();
    }  
  }

  // -----------------------------------------------------------
  // Sensor Input
  // -----------------------------------------------------------
  public boolean bottomSensorTripped(){
    return m_bottomSensor.get();
  }

  public boolean middleSensorTripped(){
    return m_middleSensor.get();
  }

  public boolean topSensorTripped(){
    return m_topSensor.get();
  }

  // Returns true if the tower is clear of balls
  public boolean allSensorsCleared() {
    if (!bottomSensorTripped() && !middleSensorTripped() && !topSensorTripped()) {
      return true;
    } else {
      return false;
    }
  }

  public void resetIndexEncoder(){
    m_towerMotor.setSelectedSensorPosition(0);
  }

  // -----------------------------------------------------------
  // Testing and Configuration
  // -----------------------------------------------------------

  // These are for calibrating the index 
  public void configIndexPower() {
    m_indexPower = SmartDashboard.getNumber("Index Power", m_indexPowerInitial);
  }

}