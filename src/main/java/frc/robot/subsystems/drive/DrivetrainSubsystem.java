package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.org.ballardrobotics.sensors.ctre.WPI_PigeonIMU;
import frc.org.ballardrobotics.sensors.fakes.FakeGyro;
import frc.org.ballardrobotics.speedcontrollers.SmartSpeedController;
import frc.org.ballardrobotics.speedcontrollers.ctre.SmartTalonFX;
import frc.org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class DrivetrainSubsystem extends SubsystemBase {
  private SmartSpeedController m_leftController, m_rightController;
  private Gyro m_gyro;

  private DifferentialDrive m_drive;

  private DifferentialDriveKinematics m_kinematics;

  private DifferentialDriveOdometry m_odometry;

  private SimpleMotorFeedforward m_feedforward;

  private Pose2d m_pose;

  private double m_heading;
  private double m_leftPosition, m_rightPosition;
  private double m_leftVelocity, m_rightVelocity;
  private double m_leftVoltage, m_rightVoltage;
  private double m_leftCurrent, m_rightCurrent;

  public static DrivetrainSubsystem create() {
    if (Robot.isReal()) {
      return createReal();
    }
    return createFake();
  }

  public static DrivetrainSubsystem createReal() {
    var leftController = new SmartTalonFX(DriveConstants.kLeftMasterDeviceID);
    var rightController = new SmartTalonFX(DriveConstants.kRightMasterDeviceID);
    var gyro = new WPI_PigeonIMU(DriveConstants.kPigeonDeviceID);
    return new DrivetrainSubsystem(leftController, rightController, gyro);
  }

  public static DrivetrainSubsystem createFake() {
    var leftController = new FakeSmartSpeedController();
    var rightController = new FakeSmartSpeedController();
    var gyro = new FakeGyro();
    return new DrivetrainSubsystem(leftController, rightController, gyro);
  }

  public DrivetrainSubsystem(SmartSpeedController leftController, SmartSpeedController rightController, Gyro gyro) {
    // NOTE: Default command is set in RobotContainer to avoid having to inject
    // joystick data.

    m_leftController = leftController;
    m_rightController = rightController;
    m_gyro = gyro;

    m_drive = new DifferentialDrive(m_leftController, m_rightController);
    m_drive.setRightSideInverted(false);
    m_drive.setDeadband(0.1);

    resetGyro();
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getAngle()));
    m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidthMeters);
    m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);
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

    m_pose = m_odometry.update(Rotation2d.fromDegrees(m_heading), m_leftPosition, m_rightPosition);

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

  private double m_prevLeftVelocity, m_prevRightVelocity;
  private double m_prevTime;
  public void setLeftRightVelocity(double leftVelocity, double rightVelocity) {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - m_prevTime;
    double leftAcceleration = 0.0;
    double rightAcceleration = 0.0;
    if (dt > 0.0 && dt < 0.1) {
      leftAcceleration = (leftVelocity - m_prevLeftVelocity) / dt;
      rightAcceleration = (rightVelocity - m_prevRightVelocity) / dt;
    }
    
    setLeftRightVelocity(
      leftVelocity,
      m_feedforward.calculate(leftVelocity, leftAcceleration),
      rightVelocity,
      m_feedforward.calculate(rightVelocity, rightAcceleration)
    );

    m_prevTime = currentTime;
    m_prevLeftVelocity = leftVelocity;
    m_prevRightVelocity = rightVelocity;
  }

  public void setLeftRightVelocity(double leftVelocity, double leftFeedforward, double rightVelocity,
      double rightFeedforward) {
    m_leftController.setVelocity(leftVelocity, leftFeedforward);
    m_rightController.setVelocity(rightVelocity, rightFeedforward);
    m_drive.feed();
  }

  public void arcadeDrive(double move, double rotate) {
    m_drive.arcadeDrive(move, rotate);
  }

  public void setPose(Pose2d pose) {
    m_pose = pose;
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(m_heading));
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void resetEncoders() {
    m_leftController.resetEncoder();
    m_rightController.resetEncoder();
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
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
