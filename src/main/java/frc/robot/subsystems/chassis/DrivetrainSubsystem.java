package frc.robot.subsystems.chassis;

import java.util.List;

import org.ballardrobotics.sensors.IMU;
import org.ballardrobotics.sensors.ctre.WPI_PigeonIMU;
import org.ballardrobotics.sensors.fakes.FakeIMU;
import org.ballardrobotics.speedcontrollers.SmartSpeedController;
import org.ballardrobotics.speedcontrollers.ctre.SmartTalonFX;
import org.ballardrobotics.speedcontrollers.fakes.FakeSmartSpeedController;
import org.ballardrobotics.subsystems.SmartSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.Robot;

public class DrivetrainSubsystem extends SubsystemBase implements SmartSubsystem {
  private SmartSpeedController m_leftController, m_rightController;
  private TransmissionSubsystem m_transmission;
  private IMU m_gyro;

  private DifferentialDrive m_drive;

  private DifferentialDriveKinematics m_kinematics;

  private DifferentialDriveOdometry m_odometry;

  private SimpleMotorFeedforward m_feedforward;

  private TrajectoryConfig m_config;
  private Trajectory m_trajectory;
  private RamseteController m_ramsete;
  private final Timer m_timer = new Timer();

  private Pose2d m_pose;
  private Twist2d m_twist;
  private Pose2d m_poseTolerance;
  private Pose2d m_poseError;
  private Pose2d m_setpoint;

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
    var leftController = new SmartTalonFX(DrivetrainConstants.kLeftMasterDeviceID);
    var rightController = new SmartTalonFX(DrivetrainConstants.kRightMasterDeviceID);

    leftController.setSensorPhase(true);

    var gyro = new WPI_PigeonIMU(DrivetrainConstants.kPigeonDeviceID);
    return new DrivetrainSubsystem(leftController, rightController, gyro);
  }

  public static DrivetrainSubsystem createFake() {
    var leftController = new FakeSmartSpeedController();
    var rightController = new FakeSmartSpeedController();
    var gyro = new FakeIMU();
    return new DrivetrainSubsystem(leftController, rightController, gyro);
  }

  public DrivetrainSubsystem(SmartSpeedController leftController, SmartSpeedController rightController, IMU gyro) {
    m_leftController = leftController;
    m_rightController = rightController;
    m_gyro = gyro;

    m_drive = new DifferentialDrive(m_leftController, m_rightController);
    m_drive.setRightSideInverted(false);
    m_drive.setDeadband(0.1);

    setTolerance(new Pose2d(new Translation2d(0.01, 0.01), 
                            new Rotation2d(1.0)));

    resetGyro();
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getAngle()));
    m_kinematics = new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidthMeters);
    m_feedforward = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV, DrivetrainConstants.kA);

    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DrivetrainConstants.ksVolts,
                                       DrivetrainConstants.kvVoltSecondsPerMeter,
                                       DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
            DrivetrainConstants.kDriveKinematics,
            10);

    m_config = new TrajectoryConfig(DrivetrainConstants.kMaxSpeedMetersPerSecond,
                                    DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DrivetrainConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);

    m_ramsete = new RamseteController();      
    
    Shuffleboard.getTab(ShuffleboardConstants.kChassisTab).add("angle", gyro);
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
    m_poseError = m_setpoint.relativeTo(m_pose);

    m_twist.dx = getLeftVelocity() + getRightVelocity() / 2;
    m_twist.dy = 0;
    m_twist.dtheta = getHeadingVelocity();

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
  // Set left and right velocity in rotations per second
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

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  public void setPose(Pose2d pose) {
    m_pose = pose;
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(m_heading));
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void setHeading(double degrees) {
    m_gyro.setAngle(degrees);
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

  public double getHeadingVelocity() {
    double currentTime = Timer.getFPGATimestamp();
    // m_prevTime is saved in setLeftRightVelocity
    double dt = currentTime - m_prevTime;
    return getHeading() / dt;
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

  // --------- Smart subsystem implementation -----------

  public void setPosition(Pose2d position){
    m_setpoint = position;
    m_trajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these interior waypoints, leave empty for now
      List.of(),
      // Pass the ending pose
      m_setpoint,
      // Pass config
      m_config
    );      
    m_timer.reset();
    m_timer.start();
  }

  public void moveToPosition() {
    double curTime = m_timer.get();

    var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
        m_ramsete.calculate(m_pose, m_trajectory.sample(curTime)));

    double leftVelocityRotations = metersToMotorRotations(targetWheelSpeeds.leftMetersPerSecond);
    double rightVelocityRotations = metersToMotorRotations(targetWheelSpeeds.rightMetersPerSecond);

    setLeftRightVelocity(leftVelocityRotations, rightVelocityRotations);
  }

  // public void setLeftRightPosition(double leftPosition, double rightPosition) {
  //   m_leftController.setProfiledPosition(leftPosition); 
  //   m_rightController.setProfiledPosition(rightPosition); 
  //   m_drive.feed();
  // }


  // In meters/radians per second
  public void setVelocity(Twist2d velocity) {
    double leftVelocityRotations = metersToMotorRotations(velocity.dx);
    double rightVelocityRotations = metersToMotorRotations(velocity.dx);

    setLeftRightVelocity(leftVelocityRotations, rightVelocityRotations);
  }

  // System State
  public Pose2d getPosition(){
    return m_pose;
  }
  public Twist2d getVelocity(){
    return m_twist;
  }

  public boolean atReference(){
    final var eTranslate = m_poseError.getTranslation();
    final var eRotate = m_poseError.getRotation();
    final var tolTranslate = m_poseTolerance.getTranslation();
    final var tolRotate = m_poseTolerance.getRotation();
    return Math.abs(eTranslate.getX()) < tolTranslate.getX()
           && Math.abs(eTranslate.getY()) < tolTranslate.getY()
           && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
  }

  // Set tolerance for the atReference method
  public void setTolerance(Pose2d poseTolerance) {
    m_poseTolerance = poseTolerance;
  }

  // Conversions
  public double metersToWheelRotations(double metersPerSecond) {
    return metersPerSecond / (DrivetrainConstants.kWheelDiameterMeters * Math.PI);
  }

  public double wheelRotationsToMotorRotations(double wheelRotations) {
      if (m_transmission.isHighGear()) {
          return wheelRotations * DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kHighGearRatio;
      }
      return wheelRotations * DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kLowGearRatio;
  }

  public double metersToMotorRotations(double meters){
    return wheelRotationsToMotorRotations(metersToWheelRotations(meters));
  }

  public double motorRotationsToWheelRotations(double motorRotations) {
      if (m_transmission.isHighGear()) {
          return motorRotations/(DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kHighGearRatio);
      }
      return motorRotations/(DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kLowGearRatio);
  }

  public double wheelRotationsToMeters(double wheelRotations) {
      return DrivetrainConstants.kWheelDiameterMeters * Math.PI * wheelRotations;
  }
}
