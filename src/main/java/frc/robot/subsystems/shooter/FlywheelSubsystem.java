package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.org.ballardrobotics.speedcontrollers.SmartSpeedController;
import frc.org.ballardrobotics.speedcontrollers.ctre.SmartTalonSRX;
import frc.org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Robot;

public class FlywheelSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  public static FlywheelSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createSimulation();
  }

  private static FlywheelSubsystem createReal() {
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

    controller.config_kP(0, 0.0);
    controller.config_kI(0, 0.0);
    controller.config_IntegralZone(0, 0);
    controller.config_kD(0, 0.0);
    controller.config_kF(0, 0.0);
    return new FlywheelSubsystem(controller);
  }

  private static FlywheelSubsystem createSimulation() {
    var controller = new FakeSmartSpeedController();
    return new FlywheelSubsystem(controller);
  }

  public FlywheelSubsystem(SmartSpeedController controller) {
    setDefaultCommand(new RunCommand(this::stop, this));
    
    m_controller = controller;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("flywheel_measured_voltage", m_controller.getMeasuredVoltage());
    SmartDashboard.putNumber("flywheel_measured_velocity", m_controller.getMeasuredVelocity());
  }

  public void stop() {
    m_controller.setVoltage(0.0);
  }

  public void setVoltage(double voltageVolts) {
    m_controller.setVoltage(voltageVolts);
  }

  public void setVelocity(double velocityRPM) {
    m_controller.setVelocity(velocityRPM);
  }
}
