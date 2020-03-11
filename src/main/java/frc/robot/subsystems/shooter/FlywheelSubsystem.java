package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import org.ballardrobotics.speedcontrollers.SmartSpeedController;
import org.ballardrobotics.speedcontrollers.ctre.SmartTalonFX;
import org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import org.ballardrobotics.types.PIDValues;
import org.ballardrobotics.types.Setpoint;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Robot;

public class FlywheelSubsystem extends SubsystemBase {
  private SmartSpeedController m_controller;

  private double m_targetVoltage, m_measuredVoltage;
  private double m_targetVelocity, m_measuredVelocity;
  private boolean m_onTarget;

  private Pose2d m_pose;
  private Twist2d m_twist;
  private Setpoint m_setpoint;

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

    m_setpoint = new Setpoint(ElevatorConstants.kSetpointTolerance); 
    m_twist = new Twist2d(0,0,0); 
  }

  @Override
  public void periodic() {
    m_targetVoltage = m_controller.getTargetVoltage();
    m_measuredVoltage = m_controller.getMeasuredVoltage();
    m_targetVelocity = m_controller.getTargetVelocity();
    m_measuredVelocity = m_controller.getMeasuredVelocity();
    m_onTarget = Math.abs(m_targetVelocity - m_measuredVelocity) < FlywheelConstants.kAcceptableVelocityErrorRPM;

    m_twist.dx = m_measuredVelocity;

    m_setpoint.updateError(m_pose, m_twist);     
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

  // --------- Smart subsystem implementation -----------

  public void setPositionReference(Pose2d position) {
    m_setpoint.create(position);
  }

  public void setPosition() {
    double xPosition = m_setpoint.getPoseSetpoint().getTranslation().getX();
    m_controller.setProfiledPosition(xPosition);
  }

  public void setVelocity(Twist2d velocity) {
    m_setpoint.create(velocity);
    double xVelocity = m_setpoint.getTwistSetpoint().dx;
    m_controller.setVelocity(xVelocity);
  }
    
  public boolean atReference(){
    return m_setpoint.atReference();
  }

  public Pose2d getPosition(){
    return m_pose;
  }
  public Twist2d getVelocity(){
    return m_twist;
  }
}
