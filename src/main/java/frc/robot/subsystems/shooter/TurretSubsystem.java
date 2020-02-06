package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.org.ballardrobotics.speedcontrollers.SmartSpeedController;
import frc.org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import frc.org.ballardrobotics.speedcontrollers.rev.SmartSparkMax;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;

public class TurretSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  private double m_targetVoltage, m_measuredVoltage;
  private double m_targetPosition, m_measuredPosition;

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
    m_targetVoltage = m_controller.getTargetVoltage();
    m_measuredVoltage = m_controller.getMeasuredVoltage();
    m_targetPosition = m_controller.getTargetPostion() * 360.0;
    m_measuredPosition = m_controller.getMeasuredPosition() * 360.0;

    SmartDashboard.putNumber("turret_target_voltage", m_targetVoltage);
    SmartDashboard.putNumber("turret_measured_voltage", m_measuredVoltage);
    SmartDashboard.putNumber("turret_target_position", m_targetPosition);
    SmartDashboard.putNumber("turret_measured_position", m_measuredPosition);
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
    double error = m_targetPosition - m_measuredPosition;
    return Math.abs(error) < TurretConstants.kAcceptablePositionErrorDeg;
  }
}
