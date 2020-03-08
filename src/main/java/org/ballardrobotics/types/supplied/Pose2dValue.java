package org.ballardrobotics.types.supplied;

import edu.wpi.first.wpilibj.geometry.Pose2d;

/**
 * PercentOutputValue stores the percent output values needed to faciliate open
 * loop control.
 */
public class Pose2dValue {
    public final Pose2d value;

    public Pose2dValue(Pose2d value) {
        this.value = value;
    }
}
