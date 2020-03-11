package org.ballardrobotics.types;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

/**
 * Class to hold the tolerances as we move the setpoint to zero
 */
public class SetpointTolerance {
    Pose2d m_position;
    Twist2d m_velocity;

    public SetpointTolerance(Pose2d position, Twist2d velocity) {
        m_position = position;
        m_velocity = velocity;
    }
  
    public void setPositionTolerance(Pose2d position) {
        m_position = position;
    }

    public void setVelocityTolerance(Twist2d velocity) {
        m_velocity = velocity;
    }
    
}
