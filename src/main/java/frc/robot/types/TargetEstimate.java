package frc.robot.types;

/**
 * Used in TargetEstimator
 */
public class TargetEstimate {
    private double m_angle;
    private double m_distance;
    private boolean m_validEstimate;

    public TargetEstimate(double angle, double distance, boolean validEstimate){
        m_angle = angle;
        m_distance = distance;
        m_validEstimate = validEstimate;
    }

    public double getAngle(){
        return m_angle;
    }

    public double getDistance(){
        return m_distance;
    }

    public boolean isValid(){
        return m_validEstimate;
    }
}
