package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.types.TargetEstimate;

/**
 * Estimates position of the target based on past data
 * Used with turret/superstructure statemachine
 */
public class TargetEstimator {
    private Pose2d m_pose;
    private double m_turretFieldAngle;
    private double m_limelightAngle;
    private double m_limelightDistance;
    private double m_updateTime;

    public TargetEstimator(){

    }

    public void update(Pose2d pose, double turretFieldAngle, double limelightAngle, double limelightDistance){
        m_updateTime = Timer.getFPGATimestamp();
    }

    public TargetEstimate getEstimate(){
        double currentTime = Timer.getFPGATimestamp(); 
        double estimatedDistance;
        double estimatedAngle;
        if(currentTime - m_updateTime > 5){
            return null;
        }



        return null;
    }
}