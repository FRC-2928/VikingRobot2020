package org.ballardrobotics.sensors.ctre;

import com.ctre.phoenix.sensors.PigeonIMU;

import org.ballardrobotics.sensors.IMU;

public class WPI_PigeonIMU extends PigeonIMU implements IMU {
    public WPI_PigeonIMU(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void close() throws Exception {
        System.err.println("UnsupportedOperation: WPI_PigeonIMU does not support close()");
    }

    @Override
    public void calibrate() {
        System.err.println("UnsupportedOperation: WPI_PigeonIMU does not support calibrate()");
    }

    @Override
    public void reset() {
        setAngle(0.0);
    }

    @Override
    public void setAngle(double angle) {
        setYaw(angle);
        setAccumZAngle(0.0);
    }

    @Override
    public double getAngle() {
        double angles[] = {0, 0, 0};
        getYawPitchRoll(angles);
        return angles[0];
    }

    @Override
    public double getRate() {
        double angleRates[] = {0, 0, 0};
        getRawGyro(angleRates);
        return angleRates[2];
    }
}
