/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
  
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Control Panel
    public static final double kControlPanelCircumference = 0.81 * Math.PI;

    public static final double kColorArcLength = kControlPanelCircumference / 8;
    
    public static final double kManipulatorCircumference = 0.0508 * Math.PI;
    
    public static double threeTurns = 26.0; // Rotate 26 segments 

    public static final class RobotMap{

        //Drivetrain
        public static final int kDrivetrainLeftFrontTalonFX = 14;
        public static final int kDrivetrainLeftBackTalonFX = 15;
        public static final int kDrivetrainRightFrontTalonFX = 1;
        public static final int kDrivetrainRightBackTalonFX = 0;

        //Shooter
        public static final int kFlywheelTalonFX = 0;

        //Turret
        public static final int kTurretSparkMax = 420;

        //Climber
      
        //Intake
        public static final int kIntakeWPI_TalonSRX = 6; //placeholder

        //Control Panel
        public static final int kControlPanelSparkMax = 69; //Placeholder

        //Solenoids
        public static final int kDrivetrainShiftSolenoid = 0; //Placeholder
        public static final int kIntakeSoleniodRightOne = 69; //place holder
        public static final int kIntakeSoleniodRightTwo = 69;
        public static final int kIntakeSoleniodLeftOne = 69;
        public static final int kIntakeSoleniodLeftTwo = 69;

        //Sensors
        public static final int kPigeonIMU = 3;
    }

    public static final class PIDConstants{
        //Control panel constants
        public static double kPanelP = 0.1;
        public static double kPanelI = 1e-4;
        public static double kPanelD = 1;
        public static double kPanelIzone = 0;
        public static double kPanelFF = 0;
        public static double kMaxOutput = 1;
        public static double kMinOutput = -1;

    }

    public static final class ConversionConstants{
        public static final double kTurretGearRatio = 236.11; 
        public static final double kTurretDegreesPerRotation = 360; 

        public static final double kFlywheelEncoderTicksPerRotation = 2048;
        public static final int kFlywheelTalonFX = 0;
        
    }
    
    public static final class OIConstants {
        public static final int kDriverControllerPort =1;
        public static final int kOperatorControllerPort =2;
    }

    public static final class DrivetrainConstants{
        public static final double kTrackWidthMeters = 69; //Placeholder
    }
}

