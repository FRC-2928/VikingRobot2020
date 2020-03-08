/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.ballardrobotics.types.PIDValues;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

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

    public static final class DrivetrainConstants {
        public static final int kLeftMasterDeviceID = 14;
        public static final int kLeftSlaveDeviceID = 15;
        public static final int kRightMasterDeviceID = 1;
        public static final int kRightSlaveDeviceID = 0;
        public static final int kPigeonDeviceID = 3;

        public static final double kHighGearRatio = 5;
        public static final double kLowGearRatio = 10.71;
        public static final double kEncoderCPR = 2048;
        public static final double kTrackWidthMeters = 0.5;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
        public static final double kUnitsPerRevolution = 1.0;

        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
		public static double kMaxSpeedMetersPerSecond = 3;
		public static double kMaxAccelerationMetersPerSecondSquared = 3;
        public static DifferentialDriveKinematics kDriveKinematics;
		public static Object kRamseteB;
		public static Object kRamseteZeta;
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
		
    }

    public static final class ArmConstants {
        public static final int kBackSolenoidPort = 4;
        public static final int kFrontSolenoidPort = 5;
    }

    public static final class ElevatorConstants {
        public static final int kControllerDeviceID = 12;
        public static final int kSolenoidChannel = 2;

        public static final double kAcceptablePositionErrorMeters = 0.02;
        public static final double kAcceptableVelocityErrorMetersPerSecond = 0.01;
    }

    public static final class TransmissionConstants {
        public static final int kSolenoidChannel = 6;
    }

    public static final class FlywheelConstants {
        public static final int kControllerDeviceID = 3;
        
        public static final double kGearRatio = 1.0;
        public static final double kUnitsPerRev = 2048;

        public static final double kAcceptableVelocityErrorRPM = 100;

        public static final PIDValues kPID = new PIDValues()
            .withP(0.04)
            .withF((1023.0 * 0.75) / 16000.0);
    }

    public static final class HoodConstants {
        public static final int kControllerDeviceID = 6;

        public static final double kGearRatio = 60.0 / 24.0;
        public static final double kUnitsPerRev = 4096.0;

        public static final double kMinAngle = 0;
        public static final double kMaxAngle = 40;

        public static final double kAcceptablePositionErrorDeg = 2.0;
        public static final double kAcceptableVelocityErrorDegPerSec = 0.5;

        public static final PIDValues kPID = new PIDValues()
            .withP(0.5)
            .withD(10.0);
        public static final double kS = 0.25; // volts
    }

    public static final class RollerConstants {
        public static final int kControllerDeviceID = 5;
    }

    public static final class TurretConstants {
        public static final int kControllerDeviceID = 9;

        public static final double kGearRatio = 169.155;
        public static final double kUnitsPerRev = 42.0;

        public static final double kMinAngle = -225.0;
        public static final double kMaxAngle = 225.0;

        public static final double kAcceptablePositionErrorDeg = 2.0;
        public static final double kAcceptableVelocityErrorDegPerSec = 0.5;

        public static final PIDValues kPID = new PIDValues()
            .withP(0.055)
            .withD(0.5);
        public static final double kS = 0.4; // volts
    }

    public static final class FeederConstants {
        public static final int kControllerDeviceID = 10;

        public static final int kTopSensorChannel = 7;
        public static final int kMiddleSensorChannel = 8;
        public static final int kBottomSensorChannel = 9;
    }

    public static final class HopperConstants {
        public static final int kControllerDeviceID = 11;
    }

    // Config Panel: http://10.29.28.11:5801
    // Stream: http://10.29.28.11:5800
    public static final class TurretLimelightConstants {
        public static final String kTableName = "limelight-turret";
        public static final int kTrackingPipeline = 0;
        public static final int kDrivePipeline = 1;
        public static final double kMountAngleDegrees = 17.5;
        public static final double kMountHeightMeters = Units.inchesToMeters(37.5);
        public static final double kTargetHeightMeters = 2.113 + (0.381 / 2); 
    }

    public static final class ShuffleboardConstants {
        public static final String kChassisTab = "Chassis";
        public static final String kIndexerTab = "Indexer";
        public static final String kIntakeTab  = "Intake";
        public static final String kManagerTab = "Managers";
        public static final String kShooterTab = "Shooter";
    }
}
