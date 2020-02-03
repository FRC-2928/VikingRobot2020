package frc.org.ballardrobotics.sensors.ctre;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * Add your docs here.
 */
public class WPI_PigeonIMU extends PigeonIMU implements Gyro {
    public WPI_PigeonIMU(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void close() throws Exception {
        // NO-OP
    }

    @Override
    public void calibrate() {
        throw new UnsupportedOperationException("calibrate() is not supported for PigeonIMU. Use the Phoenix Tuner to calibrate.");
    }

    @Override
    public void reset() {
        setYaw(0.0);
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
