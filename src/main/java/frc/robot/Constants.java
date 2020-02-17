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
        public static final int kFeederSparkMax = 4;
        public static final int kTowerVictorSPX = 987; //Placerholder

        public static final int kIRSensorBottom = 0;
        public static final int kIRSensorMiddle = 1;
        public static final int kIRSensorTop = 2;

        //Shooter
        public static final int kFlywheelTalonFX = 0;
        public static final int kHoodTalonSRX = 5;

        //Climber
        public static final int kClimberTalonFX = 5; //Placeholder
        public static final int kClimberSolenoidBrake = 420;

        //Control Panel
        public static final int kControlPanelTalonWPI = 69; //Placeholder

        //Intake
        public static final int kIntakeWPI_TalonSRX = 6; //placeholder

        //Solenoids
        public static final int kIntakeSoleniodRightOne = 69; //place holder
        public static final int kIntakeSoleniodRightTwo = 69;
        public static final int kIntakeSoleniodLeftOne = 69;
        public static final int kIntakeSoleniodLeftTwo = 69;
        public static final int kDrivetrainShiftSolenoid = 0; //Placeholder

        //Sensors
        public static final int kPigeonIMU = 3;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort =1;
        public static final int kOperatorControllerPort =2;
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
        public static final double threeTurns = 26.0; // Rotate 26 segments 
        public static final double kPanelEncoderTicksPerRotation = 4096;

         // PID constants
         public static double kPanelP = 0.1;
         public static double kPanelI = 1e-4;
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

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
        public static final double kUnitsPerRevolution = 1.0;
        
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
        public static final double kFlywheelkP = 0.075;
        public static final double kFlywheelkF = (1023.0 * 0.75) / 16000.0;
        public static final double kHoodkP = 2.5;
        public static final double kHoodkD = 15;

    }
    
    public static final class FeederConstants {

        // Motor control constants
        public static final double kIndexFastForwardPower = 0.6;
        public static final double kIndexPower = 0.4;
        public static final double kIndexReversePower = -0.4;
        public static final double kHopperFastForwardPower = 0.6;
        public static final double kHopperPower = 0.4;
        public static final double kHopperReversePower = -0.4;

    }

    public static final class ClimberConstants {

        // PID constants
        public static double kClimberP = 0.1;
        public static double kClimberI = 1e-4;
        public static double kClimberD = 1;
        public static int kClimberIzone = 0;
        public static double kClimberFF = 0;
        public static int kClimberTimeout = 0; // place holders

        public static final double kStowedPositionSetpoint = 0.0; // Meters
        public static final double kDeployedPositionSetpoint = 1.6; // Meters
        public static final double kLowPositionSetpoint = 1.0; //placeholder
        public static final double kMidPositionSetpoint = 1.2;
        public static final double kHighPositionSetpoint = 1.4;
        public static final double kCLimberPower = 0.4; // Power Percent

        // TODO figure number of clicks per meter (assuming meters are used in above Setpoints)
        public static final double kClimberEncoderTicksPerRotation = 2048; // correct!
        public static final double kClimberGearRatio = 54 / 1; // moter to intermediate pulley,correct!
        // meters climber movement per intermediate pulley rev,correct!
        public static final double kDistancePerPullyRotation = .18; // 18 cm
    }

    public static final class ConversionConstants{
    
        // Flywheel
        public static final double kFlywheelEncoderTicksPerRotation = 2048;
        public static final double kFlywheelGearRatio = 1;

        // Hood
        public static final double kHoodEncoderTicksPerRotation = 4096;
        public static final double kHoodGearRatio = 60.0/24.0; 
    }
  
}