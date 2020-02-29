package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import org.ballardrobotics.speedcontrollers.SmartSpeedController;
import org.ballardrobotics.speedcontrollers.ctre.SmartTalonSRX;
import org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Robot;

public class FlywheelSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  private double m_targetVoltage, m_measuredVoltage;
  private double m_targetVelocity, m_measuredVelocity;
  private boolean m_onTarget;

  public static FlywheelSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createFake();
  }

  public static FlywheelSubsystem createReal() {
    var controller = new SmartTalonSRX(FlywheelConstants.kControllerDeviceID);
    controller.configFactoryDefault();

    controller.setInverted(true);
    controller.setSensorPhase(true);
    controller.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    controller.configVoltageCompSaturation(12.0);
    controller.enableVoltageCompensation(true);
    controller.configNominalOutputForward(0);
    controller.configNominalOutputReverse(0);
    controller.configNeutralDeadband(0.01);
    controller.setNeutralMode(NeutralMode.Coast);
    controller.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 80, 0.04));

    controller.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
    controller.configVelocityMeasurementWindow(64);

    return new FlywheelSubsystem(controller);
  }

  public static FlywheelSubsystem createFake() {
    var controller = new FakeSmartSpeedController();
    return new FlywheelSubsystem(controller);
  }

  public FlywheelSubsystem(SmartSpeedController controller) {
    m_controller = controller;
  }

  @Override
  public void periodic() {
    m_targetVoltage = m_controller.getTargetVoltage();
    m_measuredVoltage = m_controller.getMeasuredVoltage();
    m_targetVelocity = m_controller.getTargetVelocity() * 60.0;
    m_measuredVelocity = m_controller.getMeasuredVelocity() * 60.0;
    m_onTarget = Math.abs(m_targetVelocity - m_measuredVelocity) < FlywheelConstants.kAcceptableVelocityErrorRPM;

    SmartDashboard.putNumber("flywheel_target_voltage", m_targetVoltage);
    SmartDashboard.putNumber("flywheel_measured_voltage", m_measuredVoltage);
    SmartDashboard.putNumber("flywheel_target_velocity", m_targetVelocity);
    SmartDashboard.putNumber("flywheel_measured_velocity", m_measuredVelocity);
    SmartDashboard.putBoolean("flywheel_on_target", m_onTarget);
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

  public void setVelocity(double velocityRPM) {
    m_controller.setVelocity(velocityRPM);
  }

  public double getMeasuredVelocity() {
    return m_measuredVelocity;
  }

  public double getTargetVelocity() {
    return m_targetVelocity;
  }

  public boolean atTargetVelocity() {
    double error = m_targetVelocity - m_measuredVelocity;
    return Math.abs(error) < FlywheelConstants.kAcceptableVelocityErrorRPM;
  }
}
