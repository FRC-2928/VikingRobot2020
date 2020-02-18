package frc.robot.subsystems.vision;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ballardrobotics.types.TargetEstimate;
import org.ballardrobotics.utilities.TargetEstimationUtility;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.types.LimelightData;
import frc.robot.utilities.LimelightUtility;

public class TargetEstimator extends SubsystemBase {
  private final Runnable kUpdateRunnable;

  private Pose2d m_pose;
  private LimelightData m_limelightData;
  private double m_turretAngle;
  private double m_updateTime;

  public TargetEstimator(Supplier<Pose2d> poseSupplier, Supplier<LimelightData> limelightDataSupplier, DoubleSupplier turretAngleSupplier) {
    kUpdateRunnable = () ->  {
      var limelightData = limelightDataSupplier.get();
      if (!limelightData.isTargetFound()) {
        return;
      }
      m_pose = poseSupplier.get();
      m_limelightData = limelightData;
      m_turretAngle = turretAngleSupplier.getAsDouble();
      m_updateTime = Timer.getFPGATimestamp();
    };
  }

  @Override
  public void periodic() {
    kUpdateRunnable.run();
  }

  public TargetEstimate getEstimate(Pose2d currentPose) {
    if (Timer.getFPGATimestamp() - m_updateTime > 5.0) {
      return null;
    }
    if (m_pose == null || m_limelightData == null) {
      return null;
    }

    double angle = m_pose.getRotation().getDegrees() + m_turretAngle + m_limelightData.getHorizontalAngle();
    double distance = LimelightUtility.getEstimatedDistance(m_limelightData);
    return TargetEstimationUtility.getTargetEstimate(currentPose, m_pose, distance, angle);
  }
}
