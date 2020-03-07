package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.ballardrobotics.speedcontrollers.SmartSpeedController;
import org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import org.ballardrobotics.speedcontrollers.rev.SmartSparkMax;
import org.ballardrobotics.types.PIDValues;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Robot;

public class HoodSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  private double m_targetVoltage, m_measuredVoltage;
  private double m_targetPosition, m_measuredPosition;
  private double m_measuredVelocity;
  private boolean m_onTarget;

  private enum ControlMode {
    Stopped, OpenLoop, Position
  }
  private ControlMode m_mode = ControlMode.Stopped;

  public static HoodSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createFake();
  }

  public static HoodSubsystem createReal() {
    var controller = new SmartSparkMax(HoodConstants.kControllerDeviceID, MotorType.kBrushless, HoodConstants.kGearRatio);

    controller.setSoftLimit(SoftLimitDirection.kForward, (float)HoodConstants.kMaxAngle);
    controller.setSoftLimit(SoftLimitDirection.kReverse, (float)HoodConstants.kMinAngle);

    PIDValues.displayOnShuffleboard(HoodConstants.kPID, "Turret", (values) -> {
      var pid = controller.getPIDController();
      pid.setP(HoodConstants.kPID.getP(), SmartSparkMax.kPositionSlotIdx);
      pid.setI(HoodConstants.kPID.getI(), SmartSparkMax.kPositionSlotIdx);
      pid.setIZone(HoodConstants.kPID.getIZone(), SmartSparkMax.kPositionSlotIdx);
      pid.setD(HoodConstants.kPID.getD(), SmartSparkMax.kPositionSlotIdx);
      pid.setFF(HoodConstants.kPID.getF(), SmartSparkMax.kPositionSlotIdx);
    });

    return new HoodSubsystem(controller);
  }

  public static HoodSubsystem createFake() {
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
    m_measuredVelocity = m_controller.getMeasuredVelocity() * 360.0;
    m_onTarget = Math.abs(m_targetPosition - m_measuredPosition) < HoodConstants.kAcceptablePositionErrorDeg &&
                 Math.abs(m_measuredVelocity) < HoodConstants.kAcceptableVelocityErrorDegPerSec;
  }

  public void stop() {
    m_controller.setVoltage(0.0);
    m_mode = ControlMode.Stopped;
  }

  public void setVoltage(double voltageVolts) {
    m_controller.setVoltage(voltageVolts);
    m_mode = ControlMode.OpenLoop;
  }

  public void setPosition(double degrees) {
    m_controller.setPosition(degrees / 360.0);
    m_mode = ControlMode.Position;
  }

  public double getMeasuredVoltage() {
    return m_measuredVoltage;
  }

  public double getTargetVoltage() {
    return m_targetVoltage;
  }

  public double getMeasuredPosition() {
    return m_measuredPosition;
  }

  public double getTargetPosition() {
    return m_targetPosition;
  }

  public double getMeasuredVelocity() {
    return m_measuredVelocity;
  }

  public boolean atTargetPosition() {
    return m_onTarget && m_mode == ControlMode.Position;
  }
}
