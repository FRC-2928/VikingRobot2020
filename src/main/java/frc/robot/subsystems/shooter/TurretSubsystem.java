package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.ballardrobotics.speedcontrollers.SmartSpeedController;
import org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import org.ballardrobotics.speedcontrollers.rev.SmartSparkMax;
import org.ballardrobotics.types.PIDValues;
import org.ballardrobotics.types.supplied.PercentOutputValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;
import frc.robot.commands.shooter.turret.TurretSetPercentOutputCommand;
import frc.robot.commands.shooter.turret.TurretSetPositionCommand;
import frc.robot.commands.shooter.turret.TurretStopCommand;

public class TurretSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  private double m_targetVoltage, m_measuredVoltage;
  private double m_targetPosition, m_measuredPosition;
  private double m_measuredVelocity;
  private boolean m_onTarget;

  private enum ControlMode {
    Stopped, OpenLoop, Position
  }
  private ControlMode m_mode = ControlMode.Stopped;

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
  }

  public void configureShuffleboard(ShuffleboardLayout stateLayout, ShuffleboardLayout controlLayout) {
    stateLayout.addNumber("target_voltage", this::getTargetVoltage);
    stateLayout.addNumber("measured_voltage", this::getMeasuredVoltage);
    stateLayout.addNumber("target_velocity", this::getTargetPosition);
    stateLayout.addNumber("measured_velocity", this::getMeasuredPosition);
    stateLayout.addBoolean("at_target_position", this::atTargetPosition);
    stateLayout.addString("control_mode", () -> m_mode.toString());

    var positionEntry = controlLayout.add("position", 0).getEntry();
    var openLoopEntry = controlLayout.add("percent_out", 0).getEntry();
    controlLayout.add("stop", new TurretStopCommand(this));
    controlLayout.add("enable open", new TurretSetPercentOutputCommand(this, () -> new PercentOutputValue(openLoopEntry.getDouble(0.0))));
    controlLayout.add("enable position", new TurretSetPositionCommand(this, () -> positionEntry.getDouble(0.0)));
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
    if (degrees < -TurretConstants.kMaxAngle) {
      degrees = -TurretConstants.kMaxAngle;
    }
    if (degrees > TurretConstants.kMaxAngle) {
      degrees = TurretConstants.kMaxAngle;
    }
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
