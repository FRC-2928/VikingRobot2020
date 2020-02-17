package org.ballardrobotics.utilities;

public class RotationUtility {

    /**
     * Finds a target angle that is that is at the same circular postion as the input angle
     * and closest to the currentAngle while honoring minimum and maximum angle constraints.
     * 
     * If the desired position it outside of the constraints, the closest position will be returned.
     * 
     * This function can work with degrees or radians but expects all the inputs to be in the
     * same units.
     * 
     * @param inputAngle The desired position of the mechanism.
     * @param currentAngle The current position of the mechanism.
     * @param minAngle The minimum angle the mechanism can reach.
     * @param maxAngle The maximum angle the mechanism can reach.
     * @return The target angle.
     */
    public static double getTargetAngle(double inputAngle, double currentAngle, double minAngle, double maxAngle) {
        // Start with the first angle that is larger than minAngle.
        double start = inputAngle;
        while (start >= minAngle) {
            start -= 360.0;
        }
        start += 360.0;

        // Start angle is larger than max constraint. Return which constraint is closer to target.
        if (start > maxAngle) {
            return Math.abs(start - minAngle) < Math.abs(start - maxAngle) ? minAngle : maxAngle;
        }

        // Check all valid angles to see which is closest.
        double closestAngle = start;
        double closestDistance = Double.MAX_VALUE;
        for (double angle = start; angle <= maxAngle; angle += 360.0) {
            double distance = Math.abs(angle - currentAngle);
            if (distance < closestDistance) {
                closestAngle = angle;
                closestDistance = distance;
            }
        }
        return closestAngle;
    }

}
