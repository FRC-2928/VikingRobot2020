package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConversionConstants;
import frc.robot.subsystems.shooter.ShooterHoodSubsystem;

/**
 * This class will take a distance and generate an optimal trajectory for the ball
 */
public class BallTrajectory{
    final double gravity = 32.2; //feet

    public BallTrajectory(){
        SmartDashboard.putNumber("Ball distance", 0);
        SmartDashboard.putNumber("Ball time", 0);
    }

    // Calculate velocity given distance and angle
    public double calculateVelocity(double distance, double angleDegrees) {
        // Convert to radians since that's what the Math functions use
        double theta = Math.abs(Math.toRadians(angleDegrees));

        // Distance y is the height of the target
        double y = ConversionConstants.kRelativeTargetHeight;
        double x = distance;

        // Now run the magic formular to get velocity
        double velocity = Math.sqrt(gravity*x*x/ 2*(y - Math.tan(theta)))/Math.cos(theta);
        
        return velocity;
    }

    // Calculate angle given distance and velocity
    public double calculateAngle(double distance, double velocity) {

        // Distance y is the height of the target
        double x = distance;
        double result = (x*gravity)/velocity*velocity;
        double theta = Math.asin(result)/2;
        double angleDegrees = Math.toDegrees(theta);

        return angleDegrees;
    }
}