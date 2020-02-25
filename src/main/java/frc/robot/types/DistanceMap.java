package frc.robot.types;

import java.util.HashMap;
import java.util.Map;

/**
 * Class for storing LimelightData
 */
public class DistanceMap {

    private static DistanceMap m_instance;

    public static DistanceMap getInstance() {
        if (m_instance == null) {
            m_instance = new DistanceMap();
        }
        return m_instance;
    }

    private final Map<Double, Integer> m_degrees = new HashMap<>();
    private final Map<Double, Integer> m_rpm = new HashMap<>();

    public void loadMaps() {
        // Load degrees
        m_degrees.put(0.0, 40);
        m_degrees.put(0.0, 50);
        m_degrees.put(0.0, 60);
        m_degrees.put(0.0, 60);

        // Load RPM
        m_rpm.put(0.0, 3000);
        m_rpm.put(0.0, 4000);
        m_rpm.put(0.0, 5000);
        m_rpm.put(0.0, 6000);
    }

    public double getHoodDegrees(double distance) {
        // TODO constrain distance to a range before lookup
        double degrees = m_degrees.get(distance);
        return degrees;
    }

    public double getFlywheelRPM(double distance) {
        // TODO constrain distance to a range before lookup
        double rpm = m_rpm.get(distance);
        return rpm;
    }
}    