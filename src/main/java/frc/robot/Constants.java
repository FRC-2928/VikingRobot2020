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
         // PID constants

     public static double kPanelP = 0.1;
     public static double kPanelI = 1e-4;
     public static double kPanelD = 1;
     public static double kPanelIzone = 0;
     public static double kPanelFF = 0;
     public static double kMaxOutput = 1;
     public static double kMinOutput = -1;

     public static double kClimberP = 0.1;
     public static double kClimberI = 1e-4;
     public static double kClimberD = 1;
     public static int kClimberIzone = 0;
     public static double kClimberFF = 0;

            // Control Panel
        public static final double kControlPanelCircumference = 0.81 * Math.PI;

        public static final double kColorArcLength = kControlPanelCircumference / 8;

        public static final double kManipulatorCircumference = 0.0508 * Math.PI;

        public static double threeTurns = 26.0; // Rotate 26 segments 
        //Drivetrain

        //Shooter
        public static final int kFlywheelTalonFX = 0;

        //Climber
        public static final int kClimberTalonFX = 5; //Placeholder

        //Control Panel
        public static final int kControlPanelSparkMax = 69; //Placeholder

        //Intake
        public static final int kIntakeWPI_TalonSRX = 6; //placeholder

        //Solenoids
        public static final int kIntakeSoleniodRightOne = 69; //place holder
        public static final int kIntakeSoleniodRightTwo = 69;
        public static final int kIntakeSoleniodLeftOne = 69;
        public static final int kIntakeSoleniodLeftTwo = 69;
        public static final int kClimberSolenoidBrake = 420;
        public static final int kClimberTomahawk = 420;
    }

    public static final class PIDConstants{
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
    
    }

    public static final class ConversionConstants{
        public static final double kFlywheelEncoderTicksPerRotation = 2048;
        public static final int kFlywheelTalonFX = 0;

      

        public static final double kClimberEncoderTicksPerRotation = 2048; // correct!
        public static final double kClimberGearRatio = 54 / 1; // moter to intermediate pulley,correct!
        // meters climber movement per intermediate pulley rev,correct!
        public static final double kDistancePerPullyRotation = .18; // 18 cm
    }

    public static final class Conversions{
        public static final int kFlywheelEncoderTicksPerRotation = 2048;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort =1;
        public static final int kOperatorControllerPort =2;
    }
}

