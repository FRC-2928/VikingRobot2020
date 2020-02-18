package org.ballardrobotics.types;

/**
 * Add your docs here.
 */
public class TargetEstimate {
    private double m_angle;
    private double m_distance;

    public TargetEstimate(double angle, double distance) {
        m_angle = angle;
        m_distance = distance;
    }

    public double getAngle() {
        return m_angle;
    }

    public double getDistance() {
        return m_distance;
    }
}
