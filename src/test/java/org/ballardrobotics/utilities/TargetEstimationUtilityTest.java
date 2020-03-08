package org.ballardrobotics.utilities;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Add your docs here.
 */
public class TargetEstimationUtilityTest {

    @Test
    public void testGetTargetEstimate() {
        var storedPosition = new Translation2d(-4, 3);
        double storedDistance = 5;
        double storedAngle = -36.86989764584402;

        var estimate = TargetEstimationUtility.getTargetEstimate(new Translation2d(-4, 0), storedPosition, storedDistance, storedAngle);
        assertEquals("estimate.getAngle()", 0.0, estimate.getAngle(), 0.0001);
        assertEquals("estimate.getDistance()", 4.0, estimate.getDistance(), 0.0001);

        estimate = TargetEstimationUtility.getTargetEstimate(new Translation2d(-4, -3), storedPosition, storedDistance, storedAngle);
        assertEquals("estimate.getAngle()", 36.86989764584402, estimate.getAngle(), 0.0001);
        assertEquals("estimate.getDistance()", 5.0, estimate.getDistance(), 0.0001);

        estimate = TargetEstimationUtility.getTargetEstimate(new Translation2d(0.0, 0.0), storedPosition, storedDistance, storedAngle);
        assertEquals("estimate.getAngle()", 0.0, estimate.getAngle(), 0.0001);
        assertEquals("estimate.getDistance()", 0.0, estimate.getDistance(), 0.0001);
    }
}
