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
    public static final class RobotMap{
           
        //Drivetrain

        //Shooter
        public static final int kFlywheelTalonFX = 0;

        //Control Panel
        public static final int kControlPanelTalonWPI = 69; //Placeholder

        //Solenoids
        public static final int kIntakeSoleniodRightOne = 69; //place holder
        public static final int kIntakeSoleniodRightTwo = 69;
        public static final int kIntakeSoleniodLeftOne = 69;
        public static final int kIntakeSoleniodLeftTwo = 69;
        public static final int kIntakeWPI_TalonSRX = 6; //placeholder
    }

    public static final class PIDConstants{
        public static final double kPanelP = 0.1;
        public static final double kPanelI = 1e-4;
        public static final double kPanelD = 1;
        public static final int kPanelIzone = 0;
        public static final double kPanelFF = 0;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;
        public static final int kTimeoutMs = 30;
    }

    public static final class ConversionConstants{
    
        public static final double kFlywheelEncoderTicksPerRotation = 2048;
        public static final int kFlywheelTalonFX = 0;

        // Control Panel
        public static final double kPanelEncoderTicksPerRotation = 4096;
        public static final double kControlPanelCircumference = 0.81 * Math.PI;
        public static final double kColorArcLength = kControlPanelCircumference / 8;
        public static final double kManipulatorCircumference = 0.0508 * Math.PI;
        public static final double threeTurns = 26.0; // Rotate 26 segments 
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort =1;
        public static final int kOperatorControllerPort =2;
    }
}

