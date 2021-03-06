package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.Constants.ConversionConstants;
import frc.robot.types.LimelightData;
import frc.robot.types.TargetEstimate;

/**
 * Estimates position of the target based on difference in pose
 * Used with turret/superstructure statemachine
 */
public class TargetEstimator {
    private Pose2d m_initialPose, m_pose;
    private double m_updateTime;
    private double m_xLimelight, m_yLimelight;

    public TargetEstimator(){
        m_initialPose = new Pose2d();
        m_pose = new Pose2d();

        m_updateTime = 0;
        m_xLimelight = 0;
        m_yLimelight = 0;

    }

    public void update(Pose2d pose, double turretFieldAngle, LimelightData limelightData){
        if(limelightData.getTargetFound()){
            double initialTheta = turretFieldAngle + limelightData.getHorizontalOffset();
            m_updateTime = Timer.getFPGATimestamp();
            m_initialPose = pose;
            m_xLimelight = Math.cos(Math.toRadians(initialTheta)) * limelightData.getTargetDistance();
            m_yLimelight = Math.sin(Math.toRadians(initialTheta)) * limelightData.getTargetDistance();
        }
        else{
            m_pose = pose;
        }
        
    }

    public TargetEstimate getEstimate(){
        double currentTime = Timer.getFPGATimestamp(); 
        double xEstimate;
        double yEstimate;
        double estimatedDistance;
        double estimatedAngle;
        boolean validEstimate = true;

        if(currentTime - m_updateTime > 5 || Math.abs(m_initialPose.getTranslation().getDistance(m_pose.getTranslation())) > 10){
            validEstimate = false;
        }

        xEstimate = (m_pose.getTranslation().getX() - m_initialPose.getTranslation().getX ()) + m_xLimelight;
        yEstimate = (m_pose.getTranslation().getY() - m_initialPose.getTranslation().getY ()) + m_yLimelight;
        
        estimatedAngle = Math.toDegrees(Math.atan(yEstimate/xEstimate));
        estimatedDistance = (Math.sqrt(Math.pow(xEstimate, 2) + Math.pow(yEstimate, 2))) * ConversionConstants.kMetersToFeet;
        
        return new TargetEstimate(estimatedAngle, estimatedDistance, validEstimate);
    }
}