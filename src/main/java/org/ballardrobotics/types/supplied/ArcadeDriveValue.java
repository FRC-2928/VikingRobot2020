package org.ballardrobotics.types.supplied;

/**
 * ArcadeDriveValue stores the move and rotate values needed to faciliate arcade drive.
 */
public class ArcadeDriveValue {
    public final double move;
    public final double rotate;

    public ArcadeDriveValue(double move, double rotate) {
        this.move = move;
        this.rotate = rotate;
    }
}
