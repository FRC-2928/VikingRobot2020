package org.ballardrobotics.types;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

/**
 * Class to hold the tolerances as we move the setpoint to zero
 */
public class SetpointError {
    Pose2d m_position;
    Twist2d m_velocity;

    public SetpointError(Pose2d position, Twist2d velocity) {
        m_position = position;
        m_velocity = velocity;
    }
  
    public void setPositionError(Pose2d position) {
        m_position = position;
    }

    public void setVelocityError(Twist2d velocity) {
        m_velocity = velocity;
    }

    public double getRadiansPerSecond() {
        double radians = m_velocity.dtheta;
        return radians;
    }
    
}
