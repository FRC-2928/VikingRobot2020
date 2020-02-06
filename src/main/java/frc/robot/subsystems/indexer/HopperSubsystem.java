package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.org.ballardrobotics.speedcontrollers.SmartSpeedController;
import frc.org.ballardrobotics.speedcontrollers.ctre.SmartVictorSPX;
import frc.org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import frc.robot.Constants.HopperConstants;
import frc.robot.Robot;

public class HopperSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  public static HopperSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createSimulation();
  }

  private static HopperSubsystem createReal() {
    var controller = new SmartVictorSPX(HopperConstants.kControllerDeviceID);
    return new HopperSubsystem(controller);
  }

  private static HopperSubsystem createSimulation() {
    var controller = new FakeSmartSpeedController();
    return new HopperSubsystem(controller);
  }

  public HopperSubsystem(SmartSpeedController controller) {
    setDefaultCommand(new RunCommand(this::stop, this));

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
