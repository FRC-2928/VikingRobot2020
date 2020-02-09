package frc.robot.types;

public class LimelightData {
    private boolean m_targetFound;
    private double m_horizontalAngle;
    private double m_verticalAngle;
    private double m_area;
    private double m_skew;

    public LimelightData() {
        this(false, 0, 0, 0, 0);
    }

    public LimelightData(boolean targetFound, double horizontalAngle, double verticalAngle, double area, double skew) {
        m_targetFound = targetFound;
        m_horizontalAngle = horizontalAngle;
        m_verticalAngle = verticalAngle;
        m_area = area;
        m_skew = skew;
    }

    public void setTargetFound(boolean targetFound) {
        m_targetFound = targetFound;
    }

    public void setHorizontalAngle(double angle) {
        m_horizontalAngle = angle;
    }

    public void setVerticalAngle(double angle) {
        m_verticalAngle = angle;
    }

    public void setArea(double area) {
        m_area = area;
    }

    public void setSkew(double skew) {
        m_skew = skew;
    }

    public boolean isTargetFound() {
        return m_targetFound;
    }

    public double getHorizontalAngle() {
        return m_horizontalAngle;
    }

    public double getVerticalAngle() {
        return m_verticalAngle;
    }

    public double getArea() {
        return m_area;
    }

    public double getSkew() {
        return m_skew;
    }
}
