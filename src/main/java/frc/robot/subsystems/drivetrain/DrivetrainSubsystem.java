package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;
  /**
   * DrivetrainSubsystem handles all subsystem level logic for the drivetrain.
   * Possibly also Ramsete idfk I haven't finished this class yet.
   */
import frc.robot.utilities.Pigeon;

public class DrivetrainSubsystem extends SubsystemBase {
    private WPI_TalonFX m_leftMaster, m_rightMaster;
    private WPI_TalonFX m_leftSlave, m_rightSlave;

    private Pigeon m_pigeon;

    private double m_yaw;

    private DifferentialDrive m_differentialDrive;
    private SpeedControllerGroup m_leftMotors;
    private SpeedControllerGroup m_rightMotors;

    //Drivetrain kinematics, feed it width between wheels
    private DifferentialDriveKinematics m_kinematics;
    private SimpleMotorFeedforward m_feedForward;

    //Drivetrain odometry to keep track of our position on the field
    private DifferentialDriveOdometry m_odometry;

    private Pose2d m_pose;

    public static final double kNominalVoltageVolts = 12.0;
    private double m_targetVelocityRotationsPerSecond;

    // -----------------------------------------------------------
    // Initialization
    // -----------------------------------------------------------
    public DrivetrainSubsystem() {

        m_pigeon = new Pigeon();
        m_pigeon.resetGyro();

        m_leftMaster = new WPI_TalonFX(RobotMap.kDrivetrainLeftBackTalonFX);
        m_rightMaster = new WPI_TalonFX(RobotMap.kDrivetrainRightBackTalonFX);
        m_leftSlave = new WPI_TalonFX(RobotMap.kDrivetrainLeftFrontTalonFX);
        m_rightSlave = new WPI_TalonFX(RobotMap.kDrivetrainRightFrontTalonFX);

        //Setting followers, followers don't automatically follow master's inverts so you must set the invert type to FollowMaster
        m_leftSlave.follow(m_leftMaster, FollowerType.PercentOutput);
        m_leftSlave.setInverted(InvertType.FollowMaster);
        m_rightSlave.follow(m_rightMaster, FollowerType.PercentOutput);
        m_rightSlave.setInverted(InvertType.FollowMaster);

        m_rightMaster.setInverted(InvertType.InvertMotorOutput);

        // Configure the motors
        for(TalonFX fx : new TalonFX[] {m_leftMaster, m_leftSlave, m_rightMaster, m_rightSlave}){
            //Reset settings for safety
            fx.configFactoryDefault();

            //Sets voltage compensation to 12, used for percent output
            fx.configVoltageCompSaturation(12);
            fx.enableVoltageCompensation(true);

            //Setting just in case
            fx.configNominalOutputForward(0);
            fx.configNominalOutputReverse(0);
            fx.configPeakOutputForward(1);
            fx.configPeakOutputReverse(-1);

            //Setting deadband(area required to start moving the motor) to 1%
            fx.configNeutralDeadband(0.01);

            //Set to brake mode, will brake the motor when no power is sent
            fx.setNeutralMode(NeutralMode.Brake);

            /** 
             * Setting input side current limit (amps)
             * 45 continious, 80 peak, 30 millieseconds allowed at peak
             * 40 amp breaker can support above 40 amps for a little bit
             * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds should be fine
             */
            fx.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 80, 30));

            //Either using the integrated Falcon sensor or an external one, will change if needed
            fx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        }

        // Setup speed controller groups, or not.
        // m_leftMotors = new SpeedControllerGroup(m_leftMaster, m_leftSlave);
        // m_rightMotors = new SpeedControllerGroup(m_rightMaster, m_rightSlave);
        m_differentialDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);
        m_differentialDrive.setRightSideInverted(false);

        // Setup kinematics
        m_kinematics = DrivetrainConstants.kDriveKinematics;

        // Feedforward contraints
        m_feedForward = new SimpleMotorFeedforward(DrivetrainConstants.ksVolts,
                                                DrivetrainConstants.kvVoltSecondsPerMeter,
                                                DrivetrainConstants.kaVoltSecondsSquaredPerMeter);
        
        // Setup odometry to start at position 0,0 (top left of field)
        m_yaw = m_pigeon.getYaw();
        SmartDashboard.putNumber("Initial robot yaw", m_yaw);
        Rotation2d initialHeading = new Rotation2d(m_yaw);
        m_odometry = new DifferentialDriveOdometry(initialHeading);

        // We could start it elsewhere...
        // m_pose = new Pose2d(0,0,initialHeading);
        // m_odometry = new DifferentialDriveOdometry(initialHeading, m_pose);

        // Zero the encoders
        resetEncoders();
    }        
  
    // -----------------------------------------------------------
    // Process Logic
    // -----------------------------------------------------------
    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_yaw = m_pigeon.getYaw();
        SmartDashboard.putNumber("Robot yaw", m_yaw);
        m_pose = m_odometry.update(new Rotation2d(m_yaw), getLeftEncoders(), getRightEncoders());
    }

    public void outputMetersPerSecond(double leftMetersPerSecond, double rightMetersPerSecond) {
        // Feed these to the drivetrain...
        double leftVelocityRotationsPerSecond = leftMetersPerSecond / (DrivetrainConstants.kWheelDiameterMeters * Math.PI);
        double rightVelocityRotationsPerSecond = rightMetersPerSecond / (DrivetrainConstants.kWheelDiameterMeters * Math.PI);
        
        // Calculate feedforward.  TODO get acceleration
        double leftFeedForward = m_feedForward.calculate(leftVelocityRotationsPerSecond,0.0);
        double rightFeedForward = m_feedForward.calculate(rightVelocityRotationsPerSecond,0.0);
        setDriveTrainVelocity(leftVelocityRotationsPerSecond, leftFeedForward,
                              rightVelocityRotationsPerSecond, rightFeedForward);
    }

    // -----------------------------------------------------------
    // Actuator Output
    // -----------------------------------------------------------
    public void drive(double move, double rotate, boolean squaredInputs){
        m_differentialDrive.arcadeDrive(move, rotate, squaredInputs);
    }

    public void drive(double move, double rotate){
        drive(move, rotate, true);
    }

    public void setDriveTrainVoltage(double leftVolts, double rightVolts) {
        m_leftMaster.setVoltage(leftVolts);
        m_rightMaster.setVoltage(-rightVolts);
        m_differentialDrive.feed();
    }

    // Sets velocity for the left motor
    public void setLeftVelocity(double velocityRotationsPerSecond) {
        setLeftVelocity(velocityRotationsPerSecond, 0.0);
    }

    // Sets velocity for the right motor
    public void setRightVelocity(double velocityRotationsPerSecond) {
        setRightVelocity(velocityRotationsPerSecond, 0.0);
    }

    // Sets velocity and feedforward for the left motor
    public void setLeftVelocity(double velocityRotationsPerSecond, double feedforwardVolts) {
        m_leftMaster.set(ControlMode.Velocity, (velocityRotationsPerSecond * DrivetrainConstants.kUnitsPerRevolution) / 10.0, DemandType.ArbitraryFeedForward,
                feedforwardVolts / kNominalVoltageVolts);
    }

    // Sets velocity and feedforward for the right motor
    public void setRightVelocity(double velocityRotationsPerSecond, double feedforwardVolts) {
        m_rightMaster.set(ControlMode.Velocity, (velocityRotationsPerSecond * DrivetrainConstants.kUnitsPerRevolution) / 10.0, DemandType.ArbitraryFeedForward,
                feedforwardVolts / kNominalVoltageVolts);
    }

    // Sets velocity and feedforward for the drivetrain
    public void setDriveTrainVelocity(double leftVelocity, double leftFeedforward, double rightVelocity, double rightFeedforward) {
        setLeftVelocity(leftVelocity, leftFeedforward);
        setRightVelocity(rightVelocity, rightFeedforward);
        m_differentialDrive.feed();
    }

    public void stopDrivetrain() {
        setDriveTrainVoltage(0.0, 0.0);
    }

    public void setMaxOutput(double maxOutput) {
        m_differentialDrive.setMaxOutput(maxOutput);
    }

    // -----------------------------------------------------------
    // Sensor Input
    // -----------------------------------------------------------

    // Encoder readings
    public double getLeftEncoders(){
        return m_leftMaster.getSelectedSensorPosition();
    }

    public double getRightEncoders(){
        return m_rightMaster.getSelectedSensorPosition();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoders(), getRightEncoders());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
  
    public void resetEncoders(){
        m_leftMaster.setSelectedSensorPosition(0);
        m_rightMaster.setSelectedSensorPosition(0);
    }

    // Gyro readings
    public double getHeading() {
        return Math.IEEEremainder(m_pigeon.getYaw(), 360) * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
    }

    // public double getTurnRate() {
    //     return m_pigeon.getYaw() * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
    // }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    // -----------------------------------------------------------
    // Testing and Configuration
    // -----------------------------------------------------------
}