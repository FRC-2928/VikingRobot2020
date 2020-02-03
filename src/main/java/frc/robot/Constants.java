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

    public static final class DriveConstants {
        public static final int kLeftMasterDeviceID = 0;
        public static final int kLeftSlaveDeviceID = 1;
        public static final int kRightMasterDeviceID = 2;
        public static final int kRightSlaveDeviceID = 3;

        public static final int kPigeonDeviceID = 4;
    }

    public static final class TransmissionConstants {
        public static final int kSolenoidChannel = 1;
    }

    public static final class FlywheelConstants {
        public static final int kControllerDeviceID = 0;
        
        public static final double kGearRatio = 1.0;
        public static final double kUnitsPerRev = 1.0;
    }

    public static final class HoodConstants {
        public static final int kControllerDeviceID = 1;

        public static final double kGearRatio = 1.0;
        public static final double kUnitsPerRev = 1.0;
    }

    public static final class TurretConstants {
        public static final int kControllerDeviceID = 1;

        public static final double kGearRatio = 236.11;
        public static final double kUnitsPerRev = 42.0;
    }

}
