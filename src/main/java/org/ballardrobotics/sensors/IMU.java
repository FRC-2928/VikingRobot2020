package org.ballardrobotics.sensors;

import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * Add your docs here.
 */
public interface IMU extends Gyro {
    void setAngle(double angle);
}
