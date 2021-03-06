package frc.robot.subsystems.intake;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.SmartSubsystem;;

/**
 * Feedersubsystem is responsible for feeding balls into the shoote
 * We'll use sensors to keep track of ball positions to hold some in the tower
 * We can't have too many in the hopper or they'll jam
*/

public class FeederSubsystem extends SubsystemBase implements SmartSubsystem{

  private CANSparkMax m_hopperMotor;
  private CANPIDController m_hopperPID;
  private CANSparkMax m_indexMotor;
  private CANPIDController m_indexPID;

  //IR sensors to detect ball positions
  private DigitalInput m_bottomSensor;
  private DigitalInput m_middleSensor;
  private DigitalInput m_topSensor;

  private double m_indexPower = FeederConstants.kIndexPower;

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

    m_hopperMotor = new CANSparkMax(RobotMap.kHopperSparkMax, MotorType.kBrushless);
    m_indexMotor = new CANSparkMax(RobotMap.kIndexSparkMax, MotorType.kBrushless);
    m_hopperPID = m_hopperMotor.getPIDController();
    m_indexPID = m_indexMotor.getPIDController();

    // Config hopper motor
    m_hopperMotor.restoreFactoryDefaults();
    m_hopperMotor.enableVoltageCompensation(12);
    m_hopperMotor.setIdleMode(IdleMode.kBrake);
    m_hopperMotor.setSmartCurrentLimit(35, 45, 0);
    m_hopperMotor.setInverted(false);

    // Config index motor
    m_indexMotor.restoreFactoryDefaults();
    m_indexMotor.enableVoltageCompensation(12);
    m_indexMotor.setIdleMode(IdleMode.kCoast);
    m_indexMotor.setSmartCurrentLimit(35, 45, 0);
    m_indexMotor.setInverted(true);

    m_bottomSensor = new DigitalInput(RobotMap.kIRSensorBottom);
    m_middleSensor = new DigitalInput(RobotMap.kIRSensorMiddle);
    m_topSensor = new DigitalInput(RobotMap.kIRSensorTop);

    // Set default command
    setDefaultCommand(new RunCommand(this::stop, this));    

    //Placing the indexing values on ShuffleBoard
    SmartDashboard.putNumber("Index Power", FeederConstants.kIndexPower);
    SmartDashboard.putString("Indexer State", m_indexState.name());
  }

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    // Moves incoming ball up the tower. 
    // This method will be called once per scheduler
    // runFeeder();

    //configIndexPower();

    writeStateToDashboard();
    SmartDashboard.putData("Feeder Command", this);
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
    // sensor, so stay in the INDEXING state.  States 3, 6
    if (m_indexState == IndexState.INDEXING && bottomSensorTripped()) {
      indexingAction(); // no action taken
      return;
    }

    // INDEXING with bottom sensor now cleared. 
    if (m_indexState == IndexState.INDEXING) {
      if (topSensorTripped()) {
        m_indexState = IndexState.FULL_BUT_RECEIVING; // States 7 & 9
        fullButReceivingAction(); // stop indexer, start hopper
      } else if (middleSensorTripped()) {
        m_indexState = IndexState.WAITING_TO_INDEX; // State 4
        waitingToIndexAction(); // stop indexer, start hopper
      }
      return;
    }

    // Not currently INDEXING

    // Got a new ball
    if (bottomSensorTripped()) {
      if (topSensorTripped()) {
        m_indexState = IndexState.FULL;  // States 8 & 10
        fullAction(); // stop hopper and indexer
      } 
      else {
        m_indexState = IndexState.READY_TO_INDEX; // States 2 & 5
        readyToIndexAction(); // stop hopper, start indexer
        m_indexState = IndexState.INDEXING;  // States 3 & 6
      }
    } 
    // The index is completely empty
    if (allSensorsCleared()) {
      m_indexState = IndexState.WAITING_TO_INDEX; // State 1
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
    startIndex();
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
    setIndexPower(FeederConstants.kIndexFastForwardPower);
    setHopperPower(FeederConstants.kHopperFastForwardPower);
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
  // Control Input
  // -----------------------------------------------------------
  

  public void setHopperPower(double power) {
    m_hopperPID.setReference(power, ControlType.kDutyCycle);
  }

  // Set power to the tower motor
  public void setIndexPower(double power){
    m_indexPID.setReference(power, ControlType.kDutyCycle);
  } 

  // Start the hopper
  public void startHopper() {
    m_hopperState = HopperState.FEEDING;
    setHopperPower(FeederConstants.kHopperPower);
  }

  // Stop the hopper
  public void stopHopper() {
    setHopperPower(0);
    m_hopperState = HopperState.STOPPED;
  }

  // Reverse the hopper
  public void reverseHopper() {
    m_hopperState = HopperState.REVERSED;
    setHopperPower(FeederConstants.kHopperReversePower);
  }

  // Reverse the indexer
  public void reverseIndex() {
    m_indexState = IndexState.REVERSED;
    setIndexPower(FeederConstants.kIndexReversePower);
  }

  // Start the indexer
  public void startIndex() {
    setIndexPower(m_indexPower);
  }

  // Stop the indexer
  public void stopIndex() {
    setIndexPower(0);
  }

  public void setPower(double power){
    m_hopperPID.setReference(power, ControlType.kDutyCycle);
  }

  public void setPosition(double position){
    m_hopperPID.setReference(position, ControlType.kPosition, 0, IntakeConstants.kF);
  }

  public void startFeeder(){
    m_hopperMotor.set(0.85);
    m_indexMotor.set(0.85);
  }

  public void stop() {
    stopIndex();
    stopHopper();
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

  public void setVelocity(double velocity){
  }

  public void setMotion(double position) {
  }

  // -----------------------------------------------------------
  // Sensor Input
  // -----------------------------------------------------------
  public boolean bottomSensorTripped(){
    return !m_bottomSensor.get();
  }

  public boolean middleSensorTripped(){
    return !m_middleSensor.get();
  }

  public boolean topSensorTripped(){
    return !m_topSensor.get();
  }

  // Returns true if the tower is clear of balls
  public boolean allSensorsCleared() {
    if (!bottomSensorTripped() && !middleSensorTripped() && !topSensorTripped()) {
      return true;
    } else {
      return false;
    }
  }

  public double getPosition(){
    return 0;
  }

  // TODO compute velocity
  public double getVelocity() {
    return 0;
  }
  
  // Not using encoders
  public boolean atReference(){
    return true;
  }

  // -----------------------------------------------------------
  // Testing and Configuration
  // -----------------------------------------------------------

  // These are for calibrating the index 
  public void configIndexPower() {
    m_indexPower = SmartDashboard.getNumber("Index Power", m_indexPower);
  }

  public void writeStateToDashboard() {
    SmartDashboard.putString("Indexer State", m_indexState.toString());
    SmartDashboard.putString("Hopper State", m_hopperState.toString());
    SmartDashboard.putBoolean("Bottom sensor", bottomSensorTripped());
    SmartDashboard.putBoolean("Middle sensor", middleSensorTripped());
    SmartDashboard.putBoolean("Top sensor", topSensorTripped());
  }

}