package org.ballardrobotics.types.supplied;

import edu.wpi.first.wpilibj.geometry.Twist2d;

/**
 * PercentOutputValue stores the percent output values needed to faciliate open
 * loop control.
 */
public class Twist2dValue {
    public final Twist2d value;

    public Twist2dValue(Twist2d value) {
        this.value = value;
    }
}
