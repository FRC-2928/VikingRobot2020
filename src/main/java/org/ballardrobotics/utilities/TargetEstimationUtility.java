package org.ballardrobotics.utilities;

import org.ballardrobotics.types.TargetEstimate;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * TargetEstimationUtility
 */
public class TargetEstimationUtility {

    /**
     * getTargetEstimate provides an estimation for a target distance and angle
     * given the current postion the robot, the positon at which a previous 
     * measurement was taken and the distance and angle information from that
     * previous measurement.
     * 
     * @param currentPosition The current position of the robot.
     * @param storedPosition The position of the robot during the last successful measurement.
     * @param storedDistance The distance from target during the last measurement.
     * @param storedAngle The angle from target during the last measurement.
     */
    public static TargetEstimate getTargetEstimate(Translation2d currentPosition, Translation2d storedPosition, double storedDistance, double storedAngle) {
        double x = (storedPosition.getX() - currentPosition.getX()) + (storedDistance * Math.cos(Math.toRadians(storedAngle)));
        double y = (storedPosition.getY() - currentPosition.getY()) + (storedDistance * Math.sin(Math.toRadians(storedAngle)));
        double distance = Math.sqrt(x*x + y*y);
        double angle = Math.toDegrees(Math.atan2(y, x));
        return new TargetEstimate(angle, distance);
    }

}
