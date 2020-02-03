package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.org.ballardrobotics.speedcontrollers.SmartSpeedController;
import frc.org.ballardrobotics.speedcontrollers.ctre.SmartTalonSRX;
import frc.org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import frc.robot.Constants.HoodConstants;
import frc.robot.Robot;

public class HoodSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  public static HoodSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createSimulation();
  }

  private static HoodSubsystem createReal() {
    var controller = new SmartTalonSRX(HoodConstants.kControllerDeviceID);
    return new HoodSubsystem(controller);
  }

  private static HoodSubsystem createSimulation() {
    var controller = new FakeSmartSpeedController();
    return new HoodSubsystem(controller);
  }

  public HoodSubsystem(SmartSpeedController controller) {
    setDefaultCommand(new RunCommand(this::stop, this));
    
    m_controller = controller;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("hood_measured_voltage", m_controller.getMeasuredVoltage());
    SmartDashboard.putNumber("hood_measured_position", getPosition());
  }

  public void stop() {
    m_controller.setVoltage(0.0);
  }

  public void setVoltage(double voltageVolts) {
    m_controller.setVoltage(voltageVolts);
  }

  public void setPosition(double degrees) {
    m_controller.setPosition(degrees / 360.0);
  }

  public double getPosition() {
    return m_controller.getMeasuredPosition() * 360.0;
  }
}
