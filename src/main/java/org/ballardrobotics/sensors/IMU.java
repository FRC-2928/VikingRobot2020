package org.ballardrobotics.sensors;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * Add your docs here.
 */
public interface IMU extends Gyro, Sendable {
    void setAngle(double angle);
}
