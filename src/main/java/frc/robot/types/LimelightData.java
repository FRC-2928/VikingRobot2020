package frc.robot.types;

/**
 * Class for storing LimelightData
 */
public class LimelightData {
    private double m_horizontalOffset = 0;
    private double m_verticalOffset = 0;
    private double m_targetDistance = 0;
    private double m_skew = 0;
    private boolean m_isTargetFound = false;

    public LimelightData(double horizontalOffset, double verticalOffset, double targetDistance, boolean isTargetFound,
    double skew){
        m_horizontalOffset = horizontalOffset;
        m_verticalOffset = verticalOffset;
        m_targetDistance = targetDistance;
        m_isTargetFound = isTargetFound;
        m_skew = skew;
    }

    public LimelightData(double horizontalOffset, double verticalOffset, double targetDistance, boolean isTargetFound){
        m_horizontalOffset = horizontalOffset;
        m_verticalOffset = verticalOffset;
        m_targetDistance = targetDistance;
        m_isTargetFound = isTargetFound;
    }

    public LimelightData(double horizontalOffset, double verticalOffset, double targetDistance){
        m_horizontalOffset = horizontalOffset;
        m_verticalOffset = verticalOffset;
        m_targetDistance = targetDistance;
    }

    public LimelightData(double horizontalOffset, double verticalOffset){
        m_horizontalOffset = horizontalOffset;
        m_verticalOffset = verticalOffset;
    }

    public void setHorizontalOffset(double horizontalOffeset){
        m_horizontalOffset = horizontalOffeset;
    }

    public void setVerticalOffset(double verticalOffset){
        m_verticalOffset = verticalOffset;
    }

    public void setTargetDistance(double targetDistance){
        m_targetDistance = targetDistance;
    }

    public void setTargetFound(boolean isTargetFound){
        m_isTargetFound = isTargetFound;
    }

    public double getSkew(){
        return m_skew;
    }
    
    public double getHorizontalOffset(){
        return m_horizontalOffset;
    }

    public double getVerticalOffset(){
        return m_verticalOffset;
    }

    public double getTargetDistance(){
        return m_targetDistance;
    }

    public boolean getTargetFound(){
        return m_isTargetFound;
    }
}
