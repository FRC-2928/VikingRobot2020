package org.ballardrobotics.types;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

/**
 * This class compares the required setpoint to the current position
 * and velociy and computes the error. Once the error goes to zero, within
 * a certain tolerance, the atReference method will return true.
 */
public class Setpoint {

    private enum Mode {
        POSITION, VELOCITY
    }

    SetpointTolerance m_tolerance;
    Pose2d m_poseSetpoint;
    Twist2d m_twistSetpoint;
    Pose2d m_poseError;
    Twist2d m_twistError;
    Mode m_mode;

    public Setpoint(SetpointTolerance tolerance) {
        m_tolerance = tolerance;
    }

    public void create(Pose2d position) {
        m_poseSetpoint = position;
        m_mode = Mode.POSITION;
    }

    public void create(Twist2d velocity) {
        m_twistSetpoint = velocity;
        m_mode = Mode.VELOCITY;
    }

    public Pose2d getPoseSetpoint() {
        return m_poseSetpoint;
    }

    public Twist2d getTwistSetpoint() {
        return m_twistSetpoint;
    }


    public void updateError(Pose2d position, Twist2d velocity) {
        m_poseError = m_poseSetpoint.relativeTo(position);
        m_twistError = relativeTo(velocity);
    }

    public Twist2d relativeTo(Twist2d other) {
        double eX = m_twistSetpoint.dx - other.dx;
        double eY = m_twistSetpoint.dy - other.dy;
        double eTheta = m_twistSetpoint.dtheta - other.dtheta;
        return new Twist2d(eX, eY ,eTheta);
    }

    public boolean atReference() {
        if (m_mode == Mode.POSITION) {
            final var eTranslate = m_poseError.getTranslation();
            final var eRotate = m_poseError.getRotation();
            final var tolTranslate = m_tolerance.m_position.getTranslation();
            final var tolRotate = m_tolerance.m_position.getRotation();
            return Math.abs(eTranslate.getX()) < tolTranslate.getX()
                && Math.abs(eTranslate.getY()) < tolTranslate.getY()
                && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
        }
        final var eTranslateX = m_twistError.dx;
        final var eTranslateY = m_twistError.dy;
        final var eRotate = m_twistError.dtheta;
        final var tolTranslate = m_tolerance.m_velocity;
        final var tolRotate = m_tolerance.m_velocity.dtheta;
        return Math.abs(eTranslateX) < tolTranslate.dx
            && Math.abs(eTranslateY) < tolTranslate.dy
            && Math.abs(eRotate) < tolRotate;
        
    }

    
}
