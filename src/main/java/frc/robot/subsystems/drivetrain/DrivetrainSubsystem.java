package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;
  /**
   * DrivetrainSubsystem handles all subsystem level logic for the drivetrain.
   * Possibly also Ramsete idfk I haven't finished this class yet.
   */
public class DrivetrainSubsystem extends SubsystemBase {
  private WPI_TalonFX m_leftMaster, m_rightMaster;
  private WPI_TalonFX m_leftSlave, m_rightSlave;
  private PigeonIMU m_pigeon;

  private double m_yaw, m_pitch, m_roll;

  private DifferentialDrive differentialdrive;

  //Drivetrain kinematics, feed it width between wheels
  private DifferentialDriveKinematics m_kinematics;

  //Drivetrain odometry to keep track of our position on the field
  private DifferentialDriveOdometry m_odometry;

  private Pose2d m_pose;


  public DrivetrainSubsystem() {
    m_leftMaster = new WPI_TalonFX(RobotMap.kDrivetrainLeftBackTalonFX);
    m_rightMaster = new WPI_TalonFX(RobotMap.kDrivetrainRightBackTalonFX);
    m_leftSlave = new WPI_TalonFX(RobotMap.kDrivetrainLeftFrontTalonFX);
    m_rightSlave = new WPI_TalonFX(RobotMap.kDrivetrainRightFrontTalonFX);

    m_pigeon = new PigeonIMU(RobotMap.kPigeonIMU);

    //Setting followers, followers don't automatically follow master's inverts so you must set the invert type to FollowMaster
    m_leftSlave.follow(m_leftMaster, FollowerType.PercentOutput);
    m_leftSlave.setInverted(InvertType.FollowMaster);
    m_rightSlave.follow(m_rightMaster, FollowerType.PercentOutput);
    m_rightSlave.setInverted(InvertType.FollowMaster);
    
    m_rightMaster.setInverted(InvertType.InvertMotorOutput);

    //May do SpeedControllerGroup instead, not yet sure
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

    resetEncoders();

    differentialdrive = new DifferentialDrive(m_leftMaster, m_rightMaster);
    differentialdrive.setRightSideInverted(false);
    m_kinematics = new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidthMeters);
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(getYaw()));

  }  
  
  @Override
  public void periodic() {
    m_yaw = getYaw();

    m_pose = m_odometry.update(new Rotation2d(m_yaw), getLeftEncoders(), getRightEncoders());
    
  }

  public void drive(double move, double rotate, boolean squaredInputs){
    differentialdrive.arcadeDrive(move, rotate, squaredInputs);
  }

  public void drive(double move, double rotate){
    drive(move, rotate, true);
  }

  public double getLeftEncoders(){
    return m_leftMaster.getSelectedSensorPosition();
  }

  public double getRightEncoders(){
    return m_rightMaster.getSelectedSensorPosition();
  }

  public double getYaw(){
    double ypr[] = {0, 0, 0};
    m_pigeon.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public void resetEncoders(){
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
  }
}
