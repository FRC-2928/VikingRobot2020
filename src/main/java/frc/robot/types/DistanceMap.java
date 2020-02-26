package frc.robot.types;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        m_degrees.put(1.0, 15); // WALL
        m_degrees.put(2.0, 18);
        m_degrees.put(3.0, 23);
        m_degrees.put(4.0, 27);
        m_degrees.put(5.0, 35);
        m_degrees.put(6.0, 35);
        m_degrees.put(7.0, 35);
        m_degrees.put(8.0, 35);
        m_degrees.put(9.0, 35);
        m_degrees.put(10.0, 35); // INITIATION LINE
        m_degrees.put(11.0, 35);
        m_degrees.put(12.0, 36);
        m_degrees.put(13.0, 37);
        m_degrees.put(14.0, 38);
        m_degrees.put(15.0, 39);
        m_degrees.put(16.0, 40);
        m_degrees.put(17.0, 40); // START OF TRENCH
        m_degrees.put(18.0, 40);
        m_degrees.put(19.0, 40);
        m_degrees.put(20.0, 40);
        m_degrees.put(21.0, 40);
        m_degrees.put(22.0, 40);
        m_degrees.put(23.0, 40);
        m_degrees.put(24.0, 40);
        m_degrees.put(25.0, 40); // COLOR WHEEL
        // Danger zone
        m_degrees.put(26.0, 40);
        m_degrees.put(27.0, 40);
        m_degrees.put(28.0, 40);
        m_degrees.put(29.0, 40);
        m_degrees.put(30.0, 40); // FAR TRENCH

        // Load RPM
        m_rpm.put(1.0, 3500); // WALL
        m_rpm.put(2.0, 3500);
        m_rpm.put(3.0, 3500);
        m_rpm.put(4.0, 3500);
        m_rpm.put(5.0, 3500);
        m_rpm.put(6.0, 3500);
        m_rpm.put(7.0, 3500);
        m_rpm.put(8.0, 3500);
        m_rpm.put(9.0, 3600);
        m_rpm.put(10.0, 3750); // INITIATION LINE
        m_rpm.put(11.0, 3750);
        m_rpm.put(12.0, 3850);
        m_rpm.put(13.0, 3950);
        m_rpm.put(14.0, 4050);
        m_rpm.put(15.0, 4100);
        m_rpm.put(16.0, 4200);
        m_rpm.put(17.0, 4300); // START OF TRENCH
        m_rpm.put(18.0, 4400);
        m_rpm.put(19.0, 4500);
        m_rpm.put(20.0, 5000);
        m_rpm.put(21.0, 5100);
        m_rpm.put(22.0, 5200);
        m_rpm.put(23.0, 5300);
        m_rpm.put(24.0, 5400);
        m_rpm.put(25.0, 5600); // COLOR WHEEL
        // Danger zone
        m_rpm.put(26.0, 5700);
        m_rpm.put(27.0, 5800);
        m_rpm.put(28.0, 5900);
        m_rpm.put(29.0, 6000);
        m_rpm.put(30.0, 6000); // FAR TRENCH
    }

    public double getHoodDegrees(double distance) {
        double key = Math.round(distance/12);
        double degrees = m_degrees.get(key);

        SmartDashboard.putNumber("distance", distance);
        SmartDashboard.putNumber("key", key);      
        SmartDashboard.putNumber("degrees", degrees);
        return degrees;
    }

    public double getFlywheelRPM(double distance) {
        double key = Math.round(distance/12);
        double rpm = m_rpm.get(key);

        SmartDashboard.putNumber("distance", distance);
        SmartDashboard.putNumber("key", key);      
        SmartDashboard.putNumber("RPM", rpm);
        return rpm;
    }
}    