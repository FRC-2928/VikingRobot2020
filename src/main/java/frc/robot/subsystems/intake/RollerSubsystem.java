package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.ballardrobotics.speedcontrollers.SmartSpeedController;
import org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import org.ballardrobotics.speedcontrollers.rev.SmartSparkMax;
import org.ballardrobotics.types.supplied.PercentOutputValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.intake.RollerSetPercentOutputCommand;
import frc.robot.commands.intake.RollerStopCommand;
import frc.robot.Robot;

public class RollerSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  public enum State {
    Stopped,
    Running,
  }
  private State m_state = State.Stopped;

  public static RollerSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createFake();
  }

  public static RollerSubsystem createReal() {
    var controller = new SmartSparkMax(RollerConstants.kControllerDeviceID, MotorType.kBrushless);
    return new RollerSubsystem(controller);
  }

  public static RollerSubsystem createFake() {
    var controller = new FakeSmartSpeedController();
    return new RollerSubsystem(controller);
  }

  public RollerSubsystem(SmartSpeedController controller) {
    m_controller = controller;
  }

  public void configureShuffleboard(ShuffleboardLayout stateLayout, ShuffleboardLayout controlLayout) {
    stateLayout.addNumber("target_voltage", m_controller::getTargetVoltage);
    stateLayout.addNumber("measured_voltage", m_controller::getMeasuredVoltage);
    stateLayout.addString("state", () -> m_state.toString());

    var entry = controlLayout.add("percent_out", 0.0).getEntry();
    controlLayout.add("use_percent_out", new RollerSetPercentOutputCommand(this, () -> new PercentOutputValue(entry.getDouble(0.0))));
    controlLayout.add("stop", new RollerStopCommand(this));
  }

  public void stop() {
    m_controller.setVoltage(0.0);
    m_state = State.Stopped;
  }

  public void setPercentOutput(double value) {
    m_controller.setVoltage(12.0 * value);
    m_state = State.Running;
  }

  public State getState() {
    return m_state;
  }
}
