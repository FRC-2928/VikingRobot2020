package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class RobotMap{
        
        //Drivetrain
        public static final int kDrivetrainLeftFrontTalonFX = 14;
        public static final int kDrivetrainLeftBackTalonFX = 15;
        public static final int kDrivetrainRightFrontTalonFX = 1;
        public static final int kDrivetrainRightBackTalonFX = 0;

        //Feeder
        public static int kIndexSparkMax = 10;
		public static int kHopperSparkMax = 11;

        public static final int kIRSensorBottom = 9;
        public static final int kIRSensorMiddle = 8;
        public static final int kIRSensorTop = 7;

        //Shooter
        public static final int kFlywheelTalonFX = 3;
        public static int kHoodSparkMax = 6;

        //Turret
        public static final int kTurretSparkMax = 9;

        //Climber
        public static final int kClimberMotor = 12; 

        //Control Panel
        public static int kControlPanelSparkMax = 4;

        //Intake
        public static final int kIntakeSparkMax = 5; //placeholder

        //Solenoids
        public static final int kDrivetrainShiftSolenoid = 6; 
        public static final int kIntakeArmSolenoid = 4;
        public static final int kIntakeBaseSolenoid = 5;
        public static final int kRatchetSolenoid = 1;

        //Sensors
        public static final int kPigeonIMU = 3;
		
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kUp = 0;
        public static final int kMid = 90;
        public static final int kDown = 180;
        public static final int kMidTwo = 270;
    }

    public static final class ControlPanelConstants {
        // Conversion constants
        public static final double kControlPanelCircumference = 0.81 * Math.PI;
        public static final double kColorArcLength = kControlPanelCircumference / 8;
        public static final double kManipulatorCircumference = 0.0508 * Math.PI;
        public static final double kRotationDistance = 26.0; // Rotate 26 segments 
        public static final double kRotationBuffer = 4.0; //Extra buffer for rotating control panel
        public static final double kGearRatio = 10.0;

         // PID constants
         public static double kPanelP = 0.3;
         public static double kPanelI = 0;
         public static double kPanelD = 1;
         public static double kPanelIzone = 0;
         public static double kPanelFF = 0;
         public static double kMaxOutput = 1;
         public static double kMinOutput = -1;
         public static final int kTimeoutMs = 30; 
    }

    public static final class DrivetrainConstants{

        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        public static final double kTrackWidthMeters = 0.7; //Placeholder
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackWidthMeters);

        public static final boolean kGyroReversed = true;

        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
        public static final double kUnitsPerRevolution = 1.0;
        //TODO change to correct values
        public static final double kHighGearRatio = 5;
        public static final double kLowGearRatio = 10.71;
        
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class PIDConstants{
       
        //Shooter
        public static final double kFlywheelkP = 0.055;
        public static final double kFlywheelkF = (1023.0 * 0.75) / 16000.0;

        public static final double kHoodkP = 0.5;
        public static final double kHoodkD = 10;
        public static final double kHoodkF = 0.25;

        public static double kPTurret = 0.06;
        public static double kDTurret = 0.5;
        public static double kFTurret = 0.4;
    }
    
    public static final class FlywheelConstants {
        //Setpoints in RPM
        public static final double kSetpointWall = 3000;
        public static final double kSetpointInitiationLine = 4000;
        public static final double kSetpointCloseTrench = 5000;
        public static final double kSetpointFarTrench = 6000;

        public static final double kFlywheelErrorThreshold = 75;
    }

    public static final class HoodConstants{
        public static final double kP = 0.5;
        public static final double kD = 10;
        public static final double kF = 0.25;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;


        public static final double kSetpointWall = 40;
        public static final double kSetpointInitiationLine = 50;
        public static final double kSetpointCloseTrench = 60;
        public static final double kSetpointFarTrench = 60;

        public static final double kHoodLowerLimit = 30;
        public static final double kHoodUpperLimit = 70;

        public static final double kHoodErrorThreshold = 1;
		public static double kMaxRPM = 5700;
		public static double kMaxVel = 2000;
		public static double kMinVel = 0;
		public static double kMaxAcc = 1500;
		public static double kAllowedError;
	
    }

    public static final class TurretConstants {
        public static final double kTurretErrorThreshold = 1;
        public static final double kTurretLeftLimit = 320;
        public static final double kTurretRightLimit = -320;

        public static final double kTurretClimbPosition = 0; //Placeholders
        public static final double kTurretControlPanelPosition = -90; //Placeholders
    }

    public static final class FeederConstants {

        // Motor control constants
        public static final double kIndexFastForwardPower = 0.8;
        public static final double kIndexPower = 0.5;
        public static final double kIndexReversePower = -0.8;
        public static final double kHopperFastForwardPower = 0.7;
        public static final double kHopperPower = 0.55;
        public static final double kHopperReversePower = -0.8;
    }

    public static final class ClimberConstants {

        public static double kP = 0.055;
        public static double kD = 0.5;
        public static double kF = 0.4; //Placeholders

        public static final double kStowedPositionSetpoint = 0.025; // Meters - Test this before use
        public static final double kDeployedPositionSetpoint = 1.016; // Meters
        public static final double kClimberPower = 0.4; // Power Percent

        // TODO figure number of clicks per meter (assuming meters are used in above Setpoints)
        public static final double kClimberEncoderTicksPerRotation = 2048; // correct!
        public static final double kClimberGearRatio = 35.0 / 1.0; // moter to intermediate pulley, correct!
        // meters climber movement per intermediate pulley rev, correct!
        public static final double kDistancePerPullyRotation = 0.0762; // 7.62 
		public static double kClimberErrorThreshold = 0.5; // 5 cm
    }

    public static final class IntakeConstants {
        public static final double kIntakeGearRatio = 28.0 / 1.0;
    }

    public static final class LimelightConstants{

        //Limelight names
        public static final String kDriverLimelight = "limelight-driver";
        public static final String kTurretLimelight = "limelight-turret";

        //Pipelines
        public static final int kLowLimelightTrackingPipeline = 0;
        public static final int kLowLimelightDrivePipeline = 1;
        public static final int kHighLimelightTrackingPipeline = 0;
        public static final int kHighLimelightDrivePipeline = 1;

        public static final double kHighLimelightMountAngle = 17.5;
        public static final double kHighLimelightHeight = 37.5;
        public static final double kHighGoalHeight = 90;
    }

    public static final class ConversionConstants{
        public static final double kMetersToFeet = 3.281;
    
        // Flywheel
        public static final double kFlywheelEncoderTicksPerRotation = 2048;
        public static final double kFlywheelGearRatio = 1;

        // Hood
        public static final double kHoodEncoderTicksPerRotation = 4096;
        public static final double kHoodGearRatio = (10*5) * (60.0/24.0); 

        // Turret
        public static final double kTurretGearRatio = 169.155; 
        public static final double kTurretDegreesPerRotation = 360; 
    }
  
}