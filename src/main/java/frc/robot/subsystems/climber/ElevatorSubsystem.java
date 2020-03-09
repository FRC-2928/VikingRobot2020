package frc.robot.subsystems.climber;

import org.ballardrobotics.speedcontrollers.SmartSpeedController;
import org.ballardrobotics.speedcontrollers.ctre.SmartTalonFX;
import org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import org.ballardrobotics.subsystems.SmartSubsystem;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase implements SmartSubsystem{
  private SmartSpeedController m_controller;
  private Solenoid m_solenoid;

  private double m_targetVoltage, m_measuredVoltage;
  private double m_targetPosition, m_measuredPosition;
  private double m_measuredVelocity;
  private boolean m_onTarget;
  private Pose2d m_pose;
  private Twist2d m_twist;
  private Pose2d m_poseTolerance;
  private Pose2d m_poseError;
  private Pose2d m_setpoint;

  public static ElevatorSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createFake();
  }

  public static ElevatorSubsystem createReal() {
    var controller = new SmartTalonFX(ElevatorConstants.kControllerDeviceID);
    var solenoid = new Solenoid(ElevatorConstants.kSolenoidChannel);
    return new ElevatorSubsystem(controller, solenoid);
  }

  public static ElevatorSubsystem createFake() {
    var controller = new FakeSmartSpeedController();
    var solenoid = new Solenoid(ElevatorConstants.kSolenoidChannel);
    return new ElevatorSubsystem(controller, solenoid);
  }

  public ElevatorSubsystem(SmartSpeedController controller, Solenoid solenoid) {
    m_controller = controller;
    m_solenoid = solenoid;

    setTolerance(new Pose2d(new Translation2d(0.01, 0.01), 
                            new Rotation2d(1.0)));
  }

  @Override
  public void periodic() {
    m_targetVoltage = m_controller.getTargetVoltage();
    m_measuredVoltage = m_controller.getMeasuredVoltage();
    m_targetPosition = m_controller.getTargetPostion();
    m_measuredPosition = m_controller.getMeasuredPosition();
    m_measuredVelocity = m_controller.getMeasuredVelocity();
    m_poseError = m_setpoint.relativeTo(m_pose);

    m_onTarget = Math.abs(m_targetPosition - m_measuredPosition) < ElevatorConstants.kAcceptablePositionErrorMeters &&
                 Math.abs(m_measuredVelocity) < ElevatorConstants.kAcceptableVelocityErrorMetersPerSecond;
    
                 

    SmartDashboard.putNumber("elevator_target_voltage", m_targetVoltage);
    SmartDashboard.putNumber("elevator_measured_voltage", m_measuredVoltage);
    SmartDashboard.putNumber("elevator_target_velocity", m_targetPosition);
    SmartDashboard.putNumber("elevator_measured_velocity", m_measuredPosition);
    SmartDashboard.putBoolean("elevator_on_target", m_onTarget);
  }

  public void stop() {
    m_controller.setVoltage(0.0);
    engageBrake();
  }

  public void setVoltage(double voltage) {
    disengateBrake();
    m_controller.setVoltage(voltage);
  }

  public void setPosition(double position) {
    disengateBrake();
    m_controller.setPosition(position);
  }

  public boolean atTargetPosition() {
    return m_onTarget;
  }

  private void engageBrake() {
    if (!m_solenoid.get()) {
      m_solenoid.set(true);
    }
  }

  private void disengateBrake() {
    if (m_solenoid.get()) {
      m_solenoid.set(false);
    }
  }

  // --------- Smart subsystem implementation -----------

  public void setPositionReference(Pose2d reference) {
    m_setpoint = reference;
    disengateBrake();
  }

  public void setPosition() {
    m_controller.setProfiledPosition(m_setpoint.getTranslation().getX());
  }

  // In meters/radians per second
  public void setVelocity(Twist2d velocity) {
  }
    
  public boolean atReference(){
    final var eTranslate = m_poseError.getTranslation();
    final var tolTranslate = m_poseTolerance.getTranslation();
    return Math.abs(eTranslate.getX()) < tolTranslate.getX();
  }

  // Set tolerance for the atReference method
  public void setTolerance(Pose2d poseTolerance) {
    m_poseTolerance = poseTolerance;
  }

  public Pose2d getPosition(){
    return m_pose;
  }
  public Twist2d getVelocity(){
    return m_twist;
  }
}
