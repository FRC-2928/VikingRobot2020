package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterHoodSubsystem;

/**
 * This class will take a distance and generate an optimal trajectory for the ball
 */
public class BallTrajectory extends SubsystemBase{
    double hoodAngle;
    double wheelVelocity;
    double time;
    double distance;
    double horizontalVelocity;

    ShooterHoodSubsystem m_shooterhoodsubsystem;

    public BallTrajectory(){
        m_shooterhoodsubsystem = new ShooterHoodSubsystem();

        SmartDashboard.putNumber("Ball distance", 0);
        SmartDashboard.putNumber("Ball time", 0);
    }

    public double getExitVelocity(){
        hoodAngle = m_shooterhoodsubsystem.getHoodDegrees() + 30;
        distance = SmartDashboard.getNumber("Ball distance", 0);
        time = SmartDashboard.getNumber("Ball time", 0);

        horizontalVelocity = distance/time;

        return horizontalVelocity/Math.cos(Math.toRadians(hoodAngle));
    }

    public class BallConstants{
        //Hood angle at something constants
        double kDistance3400RPM = 0;
        double kDistance3000RPM = 0;

        public BallConstants(){

        }
    }
}
