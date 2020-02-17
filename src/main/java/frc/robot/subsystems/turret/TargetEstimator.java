package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.types.TargetEstimate;

/**
 * Estimates position of the target based on pose estimation
 * Used with turret/superstructure statemachine
 */
public class TargetEstimator {
    private Pose2d m_initialPose, m_pose, kTargetPose;
    private double m_turretFieldAngle, m_limelightAngle, m_limelightDistance, m_updateTime;

    public TargetEstimator(){

    }

    public void update(Pose2d pose, double turretFieldAngle, double limelightAngle, double limelightDistance, boolean validTarget){
        m_updateTime = Timer.getFPGATimestamp();
        if(validTarget){
            m_initialPose = pose;
        }
        else{
            m_pose = pose;
        }
        
    }

    public TargetEstimate getEstimate(){
        double currentTime = Timer.getFPGATimestamp(); 
        double xError;
        double yError;
        double estimatedDistance;
        double estimatedAngle;

        if(currentTime - m_updateTime > 5 || Math.abs(m_initialPose.getTranslation().getDistance(m_pose.getTranslation())) > 10){
            return null;
        }

        estimatedDistance = m_pose.getTranslation().getDistance(kTargetPose.getTranslation());
        xError = kTargetPose.getTranslation().getX() - m_pose.getTranslation().getX();
        yError = kTargetPose.getTranslation().getY() - m_pose.getTranslation().getY();
        estimatedAngle = Math.atan(yError/xError);
        
        return new TargetEstimate(estimatedAngle, estimatedDistance);
    }
}