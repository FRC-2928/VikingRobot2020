package frc.robot.subsystems.indexer;

import org.ballardrobotics.speedcontrollers.SmartSpeedController;
import org.ballardrobotics.speedcontrollers.ctre.SmartVictorSPX;
import org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Robot;

public class HopperSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  public static HopperSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createFake();
  }

  public static HopperSubsystem createReal() {
    var controller = new SmartVictorSPX(HopperConstants.kControllerDeviceID);
    return new HopperSubsystem(controller);
  }

  public static HopperSubsystem createFake() {
    var controller = new FakeSmartSpeedController();
    return new HopperSubsystem(controller);
  }

  public HopperSubsystem(SmartSpeedController controller) {
    m_controller = controller;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("feeder_measured_voltage", m_controller.getMeasuredVoltage());
  }

  public void stop() {
    m_controller.setVoltage(0.0);
  }

  public void setVoltage(double voltageVolts) {
    m_controller.setVoltage(voltageVolts);
  }
}
