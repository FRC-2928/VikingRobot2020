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
 */
public final class Constants {
    public static final class RobotMap{
        //Drivetrain

        //Intake
        public static final int kHopperVictorSPX = 420; //Placeholder
        public static final int kTowerVictorSPX = 987; //Placerholder

        public static final int kIRSensorBottom = 0;
        public static final int kIRSensorMiddle = 1;
        public static final int kIRSensorTop = 2;

        //Shooter

        //Climber

        //Control Panel
        public static final int kControlPanelSparkMax = 69; //Placeholder

        //Solenoids
    }

 public static final class OIConstants {
     public static final int kDriverControllerPort = 1;
    }
    
  public static final class FeederConstants {

    public static final int kHopperVictorSPX = 420; //Placeholder
    public static final int kTowerVictorSPX = 987; //Placerholder
    public static final int kIRSensorBottom = 0;
    public static final int kIRSensorMiddle = 1;
    public static final int kIRSensorTop = 2;


    public static final double indexPower = 0.4;
    public static final double indexSetpoint = 100;
  }

    public static final class PIDConstants{

    }

    public static final class ConversionConstants{

    }
}
