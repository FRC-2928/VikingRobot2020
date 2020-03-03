package frc.robot.subsystems.indexer;

import org.ballardrobotics.speedcontrollers.SmartSpeedController;
import org.ballardrobotics.speedcontrollers.ctre.SmartVictorSPX;
import org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import org.ballardrobotics.types.supplied.PercentOutputValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Robot;
import frc.robot.commands.indexer.feeder.FeederSetPercentOutputCommand;

public class FeederSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  private DigitalInput m_topSensor;
  private DigitalInput m_middleSensor;
  private DigitalInput m_bottomSensor;

  public static FeederSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createFake();
  }

  public static FeederSubsystem createReal() {
    var controller = new SmartVictorSPX(FeederConstants.kControllerDeviceID);
    var topSensor = new DigitalInput(FeederConstants.kTopSensorChannel);
    var middleSensor = new DigitalInput(FeederConstants.kMiddleSensorChannel);
    var bottomSensor = new DigitalInput(FeederConstants.kBottomSensorChannel);
    return new FeederSubsystem(controller, topSensor, middleSensor, bottomSensor);
  }

  public static FeederSubsystem createFake() {
    var controller = new FakeSmartSpeedController();
    var topSensor = new DigitalInput(FeederConstants.kTopSensorChannel);
    var middleSensor = new DigitalInput(FeederConstants.kMiddleSensorChannel);
    var bottomSensor = new DigitalInput(FeederConstants.kBottomSensorChannel);
    return new FeederSubsystem(controller, topSensor, middleSensor, bottomSensor);
  }
  
  public FeederSubsystem(SmartSpeedController controller, DigitalInput topSensor, DigitalInput middleSensor, DigitalInput bottomSensor) {
    m_controller = controller;
    m_topSensor = topSensor;
    m_middleSensor = middleSensor;
    m_bottomSensor = bottomSensor;
  }

  public void configureShuffleboard(ShuffleboardLayout stateLayout, ShuffleboardLayout controlLayout) {
    stateLayout.addNumber("measured_voltage", m_controller::getMeasuredVoltage);
    stateLayout.addNumber("target_voltage", m_controller::getTargetVoltage);

    var entry = controlLayout.add("percent_out", 0).getEntry();
    controlLayout.add("use_percent_out", new FeederSetPercentOutputCommand(this, () -> new PercentOutputValue(entry.getDouble(0))));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("feeder_measured_voltage", m_controller.getMeasuredVoltage());
    SmartDashboard.putBoolean("feeder_top", hasTopBall());
    SmartDashboard.putBoolean("feeder_middle", hasMiddleBall());
    SmartDashboard.putBoolean("feeder_bottom", hasBottomBall());
  }

  public void stop() {
    m_controller.setVoltage(0.0);
  }

  public void setPercentOutput(double value) {
    m_controller.setVoltage(value * 12.0);
  }

  public boolean hasTopBall() {
    return m_topSensor.get();
  }

  public boolean hasMiddleBall() {
    return m_middleSensor.get();
  }

  public boolean hasBottomBall() {
    return m_bottomSensor.get();
  }
}