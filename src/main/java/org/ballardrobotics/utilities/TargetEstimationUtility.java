package org.ballardrobotics.utilities;

import org.ballardrobotics.types.TargetEstimate;

import edu.wpi.first.wpilibj.geometry.Pose2d;

/**
 * TargetEstimationUtiltiy
 */
public class TargetEstimationUtility {

    public static TargetEstimate getTargetEstimate(Pose2d currentPose, Pose2d storedPose, double storedDistance, double storedAngle) {
        double x = (storedPose.getTranslation().getX() - currentPose.getTranslation().getX()) + (storedDistance * Math.cos(Math.toRadians(storedAngle)));
        double y = (storedPose.getTranslation().getY() - currentPose.getTranslation().getY()) + (storedDistance * Math.sin(Math.toRadians(storedAngle)));
        double distance = Math.sqrt(x*x + y*y);
        double angle = Math.toDegrees(Math.atan2(y, x));
        return new TargetEstimate(angle, distance);
    }

}
