package frc.robot.subsystems.indexer;

import org.ballardrobotics.speedcontrollers.SmartSpeedController;
import org.ballardrobotics.speedcontrollers.ctre.SmartVictorSPX;
import org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import org.ballardrobotics.types.supplied.PercentOutputValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Robot;
import frc.robot.commands.indexer.hopper.HopperSetPercentOutputCommand;

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

  public void configureShuffleboard(ShuffleboardLayout stateLayout, ShuffleboardLayout controlLayout) {
    stateLayout.addNumber("measured_voltage", m_controller::getMeasuredVoltage);
    stateLayout.addNumber("target_voltage", m_controller::getTargetVoltage);

    var entry = controlLayout.add("percent_out", 0).getEntry();
    controlLayout.add("use_percent_out", new HopperSetPercentOutputCommand(this, () -> new PercentOutputValue(entry.getDouble(0))));
  }

  public void stop() {
    m_controller.setVoltage(0.0);
  }

  public void setPercentOutput(double value) {
    m_controller.setVoltage(value * 12.0);
  }
}
