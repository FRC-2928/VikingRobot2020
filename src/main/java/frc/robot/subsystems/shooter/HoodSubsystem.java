package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.org.ballardrobotics.speedcontrollers.SmartSpeedController;
import frc.org.ballardrobotics.speedcontrollers.ctre.SmartTalonSRX;
import frc.org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import frc.robot.Constants.HoodConstants;
import frc.robot.Robot;

public class HoodSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  private double m_targetVoltage, m_measuredVoltage;
  private double m_targetPosition, m_measuredPosition;
  private boolean m_onTarget;

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
    m_controller = controller;
  }

  @Override
  public void periodic() {
    m_targetVoltage = m_controller.getTargetVoltage();
    m_measuredVoltage = m_controller.getMeasuredVoltage();
    m_targetPosition = m_controller.getTargetPostion() * 360.0;
    m_measuredPosition = m_controller.getMeasuredPosition() * 360.0;
    m_onTarget = Math.abs(m_targetPosition - m_measuredPosition) < HoodConstants.kAcceptablePositionErrorDeg;

    SmartDashboard.putNumber("hood_target_voltage", m_targetVoltage);
    SmartDashboard.putNumber("hood_measured_voltage", m_measuredVoltage);
    SmartDashboard.putNumber("hood_target_position", m_targetPosition);
    SmartDashboard.putNumber("hood_measured_position", m_measuredPosition);
    SmartDashboard.putBoolean("hood_on_target", m_onTarget);
  }

  public void stop() {
    m_controller.setVoltage(0.0);
  }

  public void setVoltage(double voltageVolts) {
    m_controller.setVoltage(voltageVolts);
  }

  public double getMeasuredVoltage() {
    return m_measuredVoltage;
  }

  public double getTargetVoltage() {
    return m_targetVoltage;
  }

  public void setPosition(double degrees) {
    m_controller.setPosition(degrees / 360.0);
  }

  public double getMeasuredPosition() {
    return m_measuredPosition;
  }

  public double getTargetPosition() {
    return m_targetPosition;
  }

  public boolean atTargetPosition() {
    return m_onTarget;
  }
}