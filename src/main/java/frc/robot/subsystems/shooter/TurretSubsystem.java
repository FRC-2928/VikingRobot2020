package frc.robot.subsystems.shooter;

import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.org.ballardrobotics.speedcontrollers.SmartSpeedController;
import frc.org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import frc.org.ballardrobotics.speedcontrollers.rev.SmartSparkMax;
import frc.robot.Robot;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  public static TurretSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createSimulation();
  }

  public static TurretSubsystem createReal() {
    var controller = new SmartSparkMax(TurretConstants.kControllerDeviceID, MotorType.kBrushless, EncoderType.kHallSensor,
        TurretConstants.kUnitsPerRev, TurretConstants.kGearRatio);
    return new TurretSubsystem(controller);
  }

  public static TurretSubsystem createSimulation() {
    var controller = new FakeSmartSpeedController();
    return new TurretSubsystem(controller);
  }

  public TurretSubsystem(SmartSpeedController controller) {
    setDefaultCommand(new RunCommand(this::stop, this));
    
    m_controller = controller;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("turret_measured_voltage", m_controller.getMeasuredVoltage());
    SmartDashboard.putNumber("turret_measured_position", m_controller.getMeasuredPosition());
  }

  public void stop() {
    m_controller.setVoltage(0.0);
  }

  public void setVoltage(double voltageVolts) {
    m_controller.setVoltage(voltageVolts);
  }

  public void setPosition(double positionRotations) {
    m_controller.setPosition(positionRotations);
  }
}
