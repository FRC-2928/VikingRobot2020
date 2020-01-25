package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class RobotMap{
        //Drivetrain

        //Shooter
        public static final int kFlywheelTalonFX = 4;
        public static final int kHoodTalonSRX = 5;

        //Climber

        //Control Panel
        public static final int kControlPanelSparkMax = 69; //Placeholder

        //Solenoids
    }

    public static final class PIDConstants{

        //Shooter
        public static final double kFlywheelkP = 1.25;
        public static final double kFlywheelkF = 0.0467;
        public static final double kHoodkP = 2.5;
        public static final double kHoodkD = 15;

    }

    public static final class ConversionConstants{
      
        public static final double kFlywheelEncoderTicksPerRotation = 2048;
        public static final double kFlywheelGearRatio = 24.0/14.0;

        public static final double kHoodEncoderTicksPerRotation = 4096;
        public static final double kHoodGearRatio = 60.0/24.0; 
    }
}
