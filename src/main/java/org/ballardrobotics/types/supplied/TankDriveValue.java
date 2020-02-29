package org.ballardrobotics.types.supplied;

/**
 * TankDriveValue stores the left and right values needed to faciliate tank drive.
 */
public class TankDriveValue {
    public final double left;
    public final double right;

    public TankDriveValue(double left, double right) {
        this.left = left;
        this.right = right;
    }
}
