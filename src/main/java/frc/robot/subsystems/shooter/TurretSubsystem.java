package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.ballardrobotics.speedcontrollers.SmartSpeedController;
import org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import org.ballardrobotics.speedcontrollers.rev.SmartSparkMax;
import org.ballardrobotics.types.PIDValues;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;

public class TurretSubsystem extends SubsystemBase {
  public enum State {
    Idle, 
    OpenLoop,
    Tracking,
    Tracked,
  }

  public enum TrackingType {
    None,
    Setpoint,
    FaceForward,
    Correction,
    Vision,
    Estimate,
  }

  private State m_state;
  private TrackingType m_trackingType;
  private SmartSpeedController m_controller;

  private double m_targetVoltage, m_measuredVoltage;
  private double m_targetPosition, m_measuredPosition;
  private double m_measuredVelocity;

  public static TurretSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createFake();
  }

  public static TurretSubsystem createReal() {
    var controller = new SmartSparkMax(TurretConstants.kControllerDeviceID, MotorType.kBrushless, TurretConstants.kGearRatio);

    controller.setSoftLimit(SoftLimitDirection.kForward, (float)TurretConstants.kMaxAngle);
    controller.setSoftLimit(SoftLimitDirection.kReverse, (float)TurretConstants.kMinAngle);

    PIDValues.displayOnShuffleboard(TurretConstants.kPID, "Turret", (values) -> {
      var pid = controller.getPIDController();
      pid.setP(TurretConstants.kPID.getP(), SmartSparkMax.kPositionSlotIdx);
      pid.setI(TurretConstants.kPID.getI(), SmartSparkMax.kPositionSlotIdx);
      pid.setIZone(TurretConstants.kPID.getIZone(), SmartSparkMax.kPositionSlotIdx);
      pid.setD(TurretConstants.kPID.getD(), SmartSparkMax.kPositionSlotIdx);
      pid.setFF(TurretConstants.kPID.getF(), SmartSparkMax.kPositionSlotIdx);
    });

    return new TurretSubsystem(controller);
  }

  public static TurretSubsystem createFake() {
    var controller = new FakeSmartSpeedController();
    return new TurretSubsystem(controller);
  }

  public TurretSubsystem(SmartSpeedController controller) {
    m_controller = controller;
    m_state = State.Idle;
    m_trackingType = TrackingType.None;
  }

  @Override
  public void periodic() {
    m_targetVoltage = m_controller.getTargetVoltage();
    m_measuredVoltage = m_controller.getMeasuredVoltage();
    m_targetPosition = m_controller.getTargetPostion() * 360.0;
    m_measuredPosition = m_controller.getMeasuredPosition() * 360.0;
    m_measuredVelocity = m_controller.getMeasuredVelocity() * 360.0;
  }

  public void stop() {
    m_controller.setVoltage(0.0);
  }

  public void setVoltage(double voltageVolts) {
    m_controller.setVoltage(voltageVolts);
  }

  public void setPosition(double degrees) {
    if (degrees < -TurretConstants.kMaxAngle) {
      degrees = -TurretConstants.kMaxAngle;
    }
    if (degrees > TurretConstants.kMaxAngle) {
      degrees = TurretConstants.kMaxAngle;
    }
    m_controller.setPosition(degrees / 360.0);
  }

  public void setState(State state) {
    m_state = state;
  }

  public void setTrackingType(TrackingType type) {
    m_trackingType = type;
  }

  public State getState() {
    return m_state;
  }

  public TrackingType getTrackingType() {
    return m_trackingType;
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
}
