package frc.robot.utilities;

public class MathUtility {
    public static double applyDeadband(double input, double deadband) {
        if (Math.abs(input) < deadband) {
            return 0.0;
        }
        return input;
    }
}
