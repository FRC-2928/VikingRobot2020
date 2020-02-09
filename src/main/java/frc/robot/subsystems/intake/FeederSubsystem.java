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

  /**
   * Moves the ball up the tower.  There are two options for doing this
   * so we can test which one works best.
   * Comment out the one that you are not going to use.  
  */
  public void startIndex() {

    // Set state to INDEXING. This will get cleared in checkIndexState()
    m_indexState = IndexState.INDEXING;

    // Move until index sensor is tripped. This happens in checkIndexState()
    setIndexPower(m_indexPower);

    // OR

    // Run a PID loop within the Talon controller.
    //m_towerMotor.set(ControlMode.Position, m_indexSetpoint);

  }

  // Handles the indexing of balls in the tower
  public void runFeeder() {

    // Check if we should take any action on the index
    checkIndexState();

    // Still indexing so we're out of here
    if (m_indexState == IndexState.INDEXING) {
      return;
    }

    // Ball at the top but empty on the bottom - states 5 & 6
    if (m_indexState == IndexState.FULL_BUT_RECEIVING) {
      stopIndex();     
      startHopper(); // The bottom slot is empty so start the hopper
      return;
    }  

    // Ball at the top and bottom so stop the index - states 7 & 8
    if (m_indexState == IndexState.FULL) {  
      stopIndex();
      return; 
    }

    // Got a ball in the bottom so index - states 3 & 4
    if (m_indexState == IndexState.READY_TO_INDEX) {
      stopHopper(); // Prevent more balls from coming in
      startIndex(); // Move the ball up the tower
      return;
    } 

    // Bottom slot is empty so waiting for a ball - states 1 & 2
    if (m_indexState == IndexState.WAITING_TO_INDEX) {
      stopIndex(); // Index should be stopped
      startHopper(); // Get more balls in
    }

  }

  /**
   * Check the state of the index to determine what action should be taken next.
   */
  public void checkIndexState() {

    // The bottom sensor will remain tripped until the ball clears the
    // sensor, so stay in the INDEXING state.
    if (m_indexState == IndexState.INDEXING && bottomSensorTripped()) {
      return;
    }

    // INDEXING with bottom sensor now cleared.
    if (m_indexState == IndexState.INDEXING) {
      if (topSensorTripped()) {
        m_indexState = IndexState.FULL_BUT_RECEIVING; // States 5 & 6
      } else if (middleSensorTripped()) {
        m_indexState = IndexState.WAITING_TO_INDEX;
      }
      return;
    }

    // Not currently INDEXING

    // Got a new ball
    if (bottomSensorTripped()) {
      if (topSensorTripped()) {
        m_indexState = IndexState.FULL;  // states 7 & 8
      } else {
        m_indexState = IndexState.READY_TO_INDEX; // states 3 & 4
      }
    } 

    // No new ball
    if (allSensorsCleared()) {
      m_indexState = IndexState.WAITING_TO_INDEX;
    }
  }

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
  // Testing
  // -----------------------------------------------------------

  // These are for calibrating the index 
  public void configIndexPower() {
    m_indexPower = SmartDashboard.getNumber("Index Power", m_indexPowerInitial);
  }

}