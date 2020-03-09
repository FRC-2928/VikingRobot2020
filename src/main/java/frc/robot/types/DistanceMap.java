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
        m_degrees.put(5.5, 40);
        m_degrees.put(7.5, 40);
        m_degrees.put(8.5, 40);
        m_degrees.put(10.0, 40); // INITIATION LINE
        m_degrees.put(11.0, 40);
        m_degrees.put(13.5, 40);
        m_degrees.put(14.5, 40);
        m_degrees.put(16.5, 40);
        m_degrees.put(18.0, 43);
        m_degrees.put(20.0, 43);

        // Load RPM
        m_rpm.put(5.5, 3725);
        m_rpm.put(7.5, 3800);
        m_rpm.put(8.5, 3900);
        m_rpm.put(10.0, 4000); // INITIATION LINE
        m_rpm.put(11.0, 4250);
        m_rpm.put(13.5, 4400);
        m_rpm.put(14.5, 4500);
        m_rpm.put(16.5, 4700);
        m_rpm.put(18.0, 5000);
        m_rpm.put(20.0, 5400);
    }

    public double getHoodDegrees(double distance) {
        double closestOffset = Double.POSITIVE_INFINITY;
        double closestDegrees = 0;

        for(var entry : m_degrees.entrySet()){
            double entryDistance = entry.getKey();
            double entryOffset = Math.abs(entryDistance - distance);
            if(entryOffset < closestOffset){
                closestOffset = entryOffset;
                closestDegrees = entry.getValue();
            }
        }

        SmartDashboard.putNumber("Distance Map Distance(hood)", distance);
        SmartDashboard.putNumber("Distance Map Degrees", closestDegrees);
        return closestDegrees;
    }

    public double getFlywheelRPM(double distance) {
        double closestOffset = Double.POSITIVE_INFINITY;
        double closestRPM = 0;

        for(var entry : m_rpm.entrySet()){
            double entryDistance = entry.getKey();
            double entryOffset = Math.abs(entryDistance - distance);
            if(entryOffset < closestOffset){
                closestOffset = entryOffset;
                closestRPM = entry.getValue();
            }
        }

        SmartDashboard.putNumber("Distance Map Distance(flywheel)", distance);
        SmartDashboard.putNumber("Distance Map RPM", closestRPM);
        return closestRPM;
    }
}    