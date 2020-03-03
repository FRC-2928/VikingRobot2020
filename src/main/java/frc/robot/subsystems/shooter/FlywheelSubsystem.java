package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import org.ballardrobotics.speedcontrollers.SmartSpeedController;
import org.ballardrobotics.speedcontrollers.ctre.SmartTalonFX;
import org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import org.ballardrobotics.types.PIDValues;
import org.ballardrobotics.types.supplied.PercentOutputValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Robot;
import frc.robot.commands.shooter.flywheel.FlywheelSetPercentOutputCommand;
import frc.robot.commands.shooter.flywheel.FlywheelSetVelocityCommand;
import frc.robot.commands.shooter.flywheel.FlywheelStopCommand;

public class FlywheelSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  private double m_targetVoltage, m_measuredVoltage;
  private double m_targetVelocity, m_measuredVelocity;
  private boolean m_onTarget;

  private enum ControlMode {
    Stopped, OpenLoop, Velocity
  }
  private ControlMode m_mode = ControlMode.Stopped;

  public static FlywheelSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createFake();
  }

  public static FlywheelSubsystem createReal() {
    var controller = new SmartTalonFX(FlywheelConstants.kControllerDeviceID, FlywheelConstants.kGearRatio * FlywheelConstants.kUnitsPerRev);
    controller.configFactoryDefault();

    controller.setInverted(true);
    controller.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    controller.configVoltageCompSaturation(12.0);
    controller.enableVoltageCompensation(true);
    controller.configNominalOutputForward(0);
    controller.configNominalOutputReverse(0);
    controller.configNeutralDeadband(0.01);
    controller.setNeutralMode(NeutralMode.Coast);
    controller.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 80, 0.04));

    controller.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
    controller.configVelocityMeasurementWindow(16);

    PIDValues.displayOnShuffleboard(FlywheelConstants.kPID, "Flywheel", (values) -> {
      controller.config_kP(SmartTalonFX.kVelocitySlotIdx, FlywheelConstants.kPID.getP());
      controller.config_kI(SmartTalonFX.kVelocitySlotIdx, FlywheelConstants.kPID.getI());
      controller.config_IntegralZone(SmartTalonFX.kVelocitySlotIdx, (int)FlywheelConstants.kPID.getIZone());
      controller.config_kD(SmartTalonFX.kVelocitySlotIdx, FlywheelConstants.kPID.getD());
      controller.config_kF(SmartTalonFX.kVelocitySlotIdx, FlywheelConstants.kPID.getF());
    });

    return new FlywheelSubsystem(controller);
  }

  public static FlywheelSubsystem createFake() {
    var controller = new FakeSmartSpeedController();
    return new FlywheelSubsystem(controller);
  }

  public FlywheelSubsystem(SmartSpeedController controller) {
    m_controller = controller;
  }

  public void configureShuffleboard(ShuffleboardLayout stateLayout, ShuffleboardLayout controlLayout) {
    stateLayout.addNumber("target_voltage", this::getTargetVoltage);
    stateLayout.addNumber("measured_voltage", this::getMeasuredVoltage);
    stateLayout.addNumber("target_velocity", this::getTargetVelocity);
    stateLayout.addNumber("measured_velocity", this::getMeasuredVelocity);
    stateLayout.addBoolean("at_target_velocity", this::atTargetVelocity);
    stateLayout.addString("control_mode", () -> m_mode.toString());

    var velocityEntry = controlLayout.add("velocity", 0).getEntry();
    var openLoopEntry = controlLayout.add("percent_out", 0).getEntry();
    controlLayout.add("stop", new FlywheelStopCommand(this));
    controlLayout.add("enable open", new FlywheelSetPercentOutputCommand(this, () -> new PercentOutputValue(openLoopEntry.getDouble(0.0))));
    controlLayout.add("enable velocity", new FlywheelSetVelocityCommand(this, () -> velocityEntry.getDouble(0.0)));
  }

  @Override
  public void periodic() {
    m_targetVoltage = m_controller.getTargetVoltage();
    m_measuredVoltage = m_controller.getMeasuredVoltage();
    m_targetVelocity = m_controller.getTargetVelocity();
    m_measuredVelocity = m_controller.getMeasuredVelocity();
    m_onTarget = Math.abs(m_targetVelocity - m_measuredVelocity) < FlywheelConstants.kAcceptableVelocityErrorRPM;
  }

  public void stop() {
    m_controller.setVoltage(0.0);
    m_mode = ControlMode.Stopped;
  }

  public void setVoltage(double voltageVolts) {
    m_controller.setVoltage(voltageVolts);
    m_mode = ControlMode.OpenLoop;
  }

  public void setVelocity(double velocityRPM) {
    m_controller.setVelocity(velocityRPM);
    m_mode = ControlMode.Velocity;
  }

  public double getMeasuredVoltage() {
    return m_measuredVoltage;
  }

  public double getTargetVoltage() {
    return m_targetVoltage;
  }

  public double getMeasuredVelocity() {
    return m_measuredVelocity;
  }

  public double getTargetVelocity() {
    return m_targetVelocity;
  }

  public boolean atTargetVelocity() {
    return m_onTarget && m_mode == ControlMode.Velocity;
  }
}
