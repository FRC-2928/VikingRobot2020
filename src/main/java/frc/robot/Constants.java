/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.ballardrobotics.types.PIDValues;

import frc.robot.types.ShooterSetpoint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class OIConstants {
        public static final int kDriveControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class DriveConstants {
        public static final int kLeftMasterDeviceID = 0;
        public static final int kLeftSlaveDeviceID = 1;
        public static final int kRightMasterDeviceID = 2;
        public static final int kRightSlaveDeviceID = 3;
        public static final int kPigeonDeviceID = 4;

        public static final double kTrackWidthMeters = 0.5;

        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static final class ElevatorConstants {
        public static final int kControllerDeviceID = 0;
        public static final int kSolenoidChannel = 2;

        public static final double kAcceptablePositionErrorMeters = 0.02;
        public static final double kAcceptableVelocityErrorMetersPerSecond = 0.01;
    }

    public static final class TransmissionConstants {
        public static final int kSolenoidChannel = 1;
    }

    public static final class FlywheelConstants {
        public static final int kControllerDeviceID = 4;
        
        public static final double kGearRatio = 1.0;
        public static final double kUnitsPerRev = 1.0;

        public static final double kAcceptableVelocityErrorRPM = 100;
    }

    public static final class HoodConstants {
        public static final int kControllerDeviceID = 5;

        public static final double kGearRatio = 1.0;
        public static final double kUnitsPerRev = 1.0;

        public static final double kAcceptablePositionErrorDeg = 2.0;
        public static final double kAcceptableVelocityErrorDegPerSec = 0.5;
    }

    public static final class TurretConstants {
        public static final int kControllerDeviceID = 4;

        public static final double kGearRatio = 236.11;
        public static final double kUnitsPerRev = 42.0;

        public static final double kMaxAngle = 225.0;

        public static final double kAcceptablePositionErrorDeg = 2.0;
        public static final double kAcceptableVelocityErrorDegPerSec = 0.5;

        public static final PIDValues kPID = new PIDValues()
            .withP(0.05)
            .withD(0);
    }

    public static final class FeederConstants {
        public static final int kControllerDeviceID = 1;

        public static final int kTopSensorChannel = 0;
        public static final int kMiddleSensorChannel = 1;
        public static final int kBottomSensorChannel = 2;
    }

    public static final class HopperConstants {
        public static final int kControllerDeviceID = 1;
    }

    public static final class ShooterSetpoints {
        public static final ShooterSetpoint kInitiationLine = new ShooterSetpoint(30, 0, 5000);
    }

}
