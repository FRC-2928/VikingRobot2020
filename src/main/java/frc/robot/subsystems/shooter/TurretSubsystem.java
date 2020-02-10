package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.org.ballardrobotics.speedcontrollers.SmartSpeedController;
import frc.org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import frc.org.ballardrobotics.speedcontrollers.rev.SmartSparkMax;
import frc.org.ballardrobotics.types.PIDValues;
import frc.robot.Robot;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  private double m_targetVoltage, m_measuredVoltage;
  private double m_targetPosition, m_measuredPosition;
  private double m_measuredVelocity;
  private boolean m_onTarget;

  public static TurretSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createFake();
  }

  public static TurretSubsystem createReal() {
    var controller = new SmartSparkMax(TurretConstants.kControllerDeviceID, MotorType.kBrushless, EncoderType.kHallSensor,
        TurretConstants.kUnitsPerRev, TurretConstants.kGearRatio);

    controller.setSoftLimit(SoftLimitDirection.kForward, (float)TurretConstants.kMaxAngle);
    controller.setSoftLimit(SoftLimitDirection.kReverse, (float)-TurretConstants.kMaxAngle);

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
    var defaultCommand = new RunCommand(this::stop, this);
    defaultCommand.setName("TurretStopCommand");
    setDefaultCommand(defaultCommand);

    m_controller = controller;
  }

  @Override
  public void periodic() {
    m_targetVoltage = m_controller.getTargetVoltage();
    m_measuredVoltage = m_controller.getMeasuredVoltage();
    m_targetPosition = m_controller.getTargetPostion() * 360.0;
    m_measuredPosition = m_controller.getMeasuredPosition() * 360.0;
    m_measuredVelocity = m_controller.getMeasuredVelocity() * 360.0;
    m_onTarget = Math.abs(m_targetPosition - m_measuredPosition) < TurretConstants.kAcceptablePositionErrorDeg &&
                 Math.abs(m_measuredVelocity) < TurretConstants.kAcceptableVelocityErrorDegPerSec;

    SmartDashboard.putNumber("turret_target_voltage", m_targetVoltage);
    SmartDashboard.putNumber("turret_measured_voltage", m_measuredVoltage);
    SmartDashboard.putNumber("turret_target_position", m_targetPosition);
    SmartDashboard.putNumber("turret_measured_position", m_measuredPosition);
    SmartDashboard.putNumber("turret_measured_velocity", m_measuredVelocity);
    SmartDashboard.putBoolean("turret_on_target", m_onTarget);
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
    if (degrees < -TurretConstants.kMaxAngle) {
      degrees = -TurretConstants.kMaxAngle;
    }
    if (degrees > TurretConstants.kMaxAngle) {
      degrees = TurretConstants.kMaxAngle;
    }
    m_controller.setPosition(degrees / 360.0);
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
    return m_onTarget;
  }
}
