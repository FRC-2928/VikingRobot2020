package frc.robot.utilities;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import frc.robot.Constants.RobotMap;

/**
 * Utility class responsible for the gyro
 */
public class Pigeon{
    private PigeonIMU m_pigeon;

    public Pigeon(){
        m_pigeon = new PigeonIMU(RobotMap.kPigeonIMU);
        m_pigeon.enterCalibrationMode(CalibrationMode.BootTareGyroAccel, 100);
    }

    public double getYaw(){
        double ypr[] = {0, 0, 0};
        m_pigeon.getYawPitchRoll(ypr);
        return ypr[0];
    }

    public void resetGyro(){
        m_pigeon.setYaw(0);
    }
}
