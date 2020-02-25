package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.Timer;
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
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_targetVelocityRotationsPerSecond;

    private double m_leftPosition, m_rightPosition;
    private Supplier<TransmissionSubsystem.GearState> m_gearStateSupplier;
    private double m_prevLeftEncoder, m_prevRightEncoder;
    private double m_prevSetOutputTime; 

    private double m_leftVelocity, m_rightVelocity; 

    // -----------------------------------------------------------
    // Initialization
    // -----------------------------------------------------------
    public DrivetrainSubsystem(Supplier<TransmissionSubsystem.GearState> gearStateSupplier) {

        m_gearStateSupplier = gearStateSupplier;

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
            fx.setNeutralMode(NeutralMode.Coast);

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
        
        // Save previous wheel speeds. Start at zero.
        m_prevSpeeds = new DifferentialDriveWheelSpeeds(0,0);

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
    
        double leftEncoderCount = m_leftMaster.getSelectedSensorPosition();
        double rightEncoderCount = m_rightMaster.getSelectedSensorPosition();
        double deltaLeftCount = leftEncoderCount - m_prevLeftEncoder;
        double deltaRightCount = rightEncoderCount - m_prevRightEncoder;

        var gearState = m_gearStateSupplier.get();
        m_leftPosition += wheelRotationsToMeters(motorRotationsToWheelRotations(deltaLeftCount, gearState));
        m_rightPosition += wheelRotationsToMeters(motorRotationsToWheelRotations(deltaRightCount, gearState));

        double leftEncoderVelocity = m_leftMaster.getSelectedSensorVelocity();
        double rightEncoderVelocity = m_rightMaster.getSelectedSensorVelocity();
        m_leftVelocity = wheelRotationsToMeters(motorRotationsToWheelRotations(leftEncoderVelocity, gearState)) * 10;
        m_rightVelocity = wheelRotationsToMeters(motorRotationsToWheelRotations(rightEncoderVelocity, gearState)) * 10;

        // Update the odometry in the periodic block
        m_yaw = m_pigeon.getYaw();
        m_pose = m_odometry.update(Rotation2d.fromDegrees(m_yaw), m_leftPosition, m_rightPosition);

        //Stores current values for next run through
        m_prevLeftEncoder = leftEncoderCount;
        m_prevRightEncoder = rightEncoderCount;

        SmartDashboard.putNumber("Left Wheel Position", m_leftPosition);
        SmartDashboard.putNumber("Right Wheel Position", m_rightPosition);
        SmartDashboard.putNumber("Left Wheel Speed", m_leftVelocity);
        SmartDashboard.putNumber("Right Wheel Speed", m_rightVelocity);
        SmartDashboard.putNumber("Robot yaw", m_yaw);
    }

    public double metersToWheelRotations(double metersPerSecond) {
        return metersPerSecond / (DrivetrainConstants.kWheelDiameterMeters * Math.PI);
    }

    public double wheelRotationsToMotorRotations(double wheelRotations, TransmissionSubsystem.GearState gearState) {
        if (gearState == TransmissionSubsystem.GearState.HIGH) {
            return wheelRotations * DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kHighGearRatio;
        }
        return wheelRotations * DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kLowGearRatio;
    }

    public double motorRotationsToWheelRotations(double motorRotations, TransmissionSubsystem.GearState gearState) {
        if (gearState == TransmissionSubsystem.GearState.HIGH) {
            return motorRotations/(DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kHighGearRatio);
        }
        return motorRotations/(DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kLowGearRatio);
    }

    public double wheelRotationsToMeters(double wheelRotations) {
        return DrivetrainConstants.kWheelDiameterMeters * Math.PI * wheelRotations;
    }

    // -----------------------------------------------------------
    // Actuator Output
    // -----------------------------------------------------------
    public void drive(DoubleSupplier move, DoubleSupplier rotate){
        drive(move.getAsDouble(), rotate.getAsDouble(), true);
    }

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

    public void setOutputMetersPerSecond(double leftMetersPerSecond, double rightMetersPerSecond) {
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - m_prevSetOutputTime;
        double leftAcceleration = 0;
        double rightAcceleration = 0;
        
        if (deltaTime > 0 && deltaTime < 0.1) {
            leftAcceleration = (leftMetersPerSecond - m_prevSpeeds.leftMetersPerSecond)/deltaTime;
            rightAcceleration = (rightMetersPerSecond - m_prevSpeeds.rightMetersPerSecond)/deltaTime;
        }

        // Calculate feedforward for the left and right wheels.
        double leftFeedForward = m_feedForward.calculate(leftMetersPerSecond, leftAcceleration);
        double rightFeedForward = m_feedForward.calculate(rightMetersPerSecond, rightAcceleration);
        
        // Convert meters per second to rotations per second
        var gearState = m_gearStateSupplier.get();
        double leftVelocityTicksPerSec = wheelRotationsToMotorRotations(metersToWheelRotations(leftMetersPerSecond), gearState);
        double rightVelocityTicksPerSec = wheelRotationsToMotorRotations(metersToWheelRotations(leftMetersPerSecond), gearState);

        m_leftMaster.set(ControlMode.Velocity, leftVelocityTicksPerSec/10.0, DemandType.ArbitraryFeedForward, leftFeedForward/12.0);
        m_rightMaster.set(ControlMode.Velocity, rightVelocityTicksPerSec/10.0, DemandType.ArbitraryFeedForward, rightFeedForward/12.0);

        // Save previous speeds
        m_prevSpeeds.leftMetersPerSecond = leftMetersPerSecond;
        m_prevSpeeds.rightMetersPerSecond = rightMetersPerSecond;

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

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftVelocity, m_rightVelocity);
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

    public double getLeftPosition() {
        return m_leftPosition;
    }

    public double getRightPosition() {
        return m_rightPosition;
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    // -----------------------------------------------------------
    // Testing and Configuration
    // -----------------------------------------------------------
}
