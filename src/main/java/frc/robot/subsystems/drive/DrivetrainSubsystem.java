package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.org.ballardrobotics.sensors.ctre.WPI_PigeonIMU;
import frc.org.ballardrobotics.sensors.fakes.FakeGyro;
import frc.org.ballardrobotics.speedcontrollers.SmartSpeedController;
import frc.org.ballardrobotics.speedcontrollers.ctre.SmartTalonFX;
import frc.org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  private SmartSpeedController m_leftController, m_rightController;
  private Gyro m_gyro;

  private DifferentialDrive m_drive;

  private double m_heading;
  private double m_leftPosition, m_rightPosition;
  private double m_leftVelocity, m_rightVelocity;
  private double m_leftVoltage, m_rightVoltage;
  private double m_leftCurrent, m_rightCurrent;

  public static DrivetrainSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createSimulation();
  }

  private static DrivetrainSubsystem createReal() {
    var leftController = new SmartTalonFX(DriveConstants.kLeftMasterDeviceID);
    var rightController = new SmartTalonFX(DriveConstants.kRightMasterDeviceID);
    var gyro = new WPI_PigeonIMU(DriveConstants.kPigeonDeviceID);
    return new DrivetrainSubsystem(leftController, rightController, gyro);
  }

  private static DrivetrainSubsystem createSimulation() {
    var leftController = new FakeSmartSpeedController();
    var rightController = new FakeSmartSpeedController();
    var gyro = new FakeGyro();
    return new DrivetrainSubsystem(leftController, rightController, gyro);
  }
  
  public DrivetrainSubsystem(SmartSpeedController leftController, SmartSpeedController rightController, Gyro gyro) {
    setDefaultCommand(new RunCommand(this::stop, this));
    
    m_leftController = leftController;
    m_rightController = rightController;
    m_gyro = gyro;
    m_drive = new DifferentialDrive(m_leftController, m_rightController);
    m_drive.setRightSideInverted(false);
  }

  @Override
  public void periodic() {
    m_heading = m_gyro.getAngle();

    m_leftPosition = m_leftController.getMeasuredPosition();
    m_rightPosition = m_rightController.getMeasuredPosition();

    m_leftVelocity = m_leftController.getMeasuredVelocity();
    m_rightVelocity = m_rightController.getMeasuredVelocity();

    m_leftVoltage = m_leftController.getMeasuredVoltage();
    m_rightVoltage = m_rightController.getMeasuredVoltage();

    m_leftCurrent = m_leftController.getMeasuredCurrent();
    m_rightCurrent = m_rightController.getMeasuredCurrent();

    SmartDashboard.putNumber("drive_heading", m_heading);
    SmartDashboard.putNumber("drive_left_pos", m_leftPosition);
    SmartDashboard.putNumber("drive_right_pos", m_rightPosition);
    SmartDashboard.putNumber("drive_left_vel", m_leftVelocity);
    SmartDashboard.putNumber("drive_right_vel", m_rightVelocity);
    SmartDashboard.putNumber("drive_left_voltage", m_leftVoltage);
    SmartDashboard.putNumber("drive_right_voltage", m_rightVoltage);
    SmartDashboard.putNumber("drive_left_current", m_leftCurrent);
    SmartDashboard.putNumber("drive_right_current", m_rightCurrent);
  }

  public void stop() {
    setLeftRightVoltage(0.0, 0.0);
  }

  public void setLeftRightVoltage(double leftVoltage, double rightVoltage) {
    m_leftController.setVoltage(leftVoltage);
    m_rightController.setVoltage(rightVoltage);
    m_drive.feed();
  }

  public void setLeftRightVelocity(double leftVelocity, double leftFeedforward, double rightVelocity, double rightFeedforward) {
    m_leftController.setVelocity(leftVelocity, leftFeedforward);
    m_rightController.setVelocity(rightVelocity, rightFeedforward);
    m_drive.feed();
  }

  public void arcadeDrive(double move, double rotate) {
    m_drive.arcadeDrive(move, rotate);
  }

  public double getHeading() {
    return m_heading;
  }

  public double getLeftPosition() {
    return m_leftPosition;
  }

  public double getRightPosition() {
    return m_rightPosition;
  }

  public double getLeftVelocity() {
    return m_leftVelocity;
  }

  public double getRightVelocity() {
    return m_rightVelocity;
  }

  public double getLeftVoltage() {
    return m_leftVoltage;
  }

  public double getRightVoltage() {
    return m_rightVoltage;
  }

  public double getLeftCurrent() {
    return m_leftCurrent;
  }

  public double getRightCurrent() {
    return m_rightCurrent;
  }
}
