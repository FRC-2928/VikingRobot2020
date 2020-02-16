package frc.robot.subsystems.vision;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.types.LimelightData;
import frc.robot.types.TargetEstimate;

public class TargetEstimator extends SubsystemBase {
  private final Supplier<Pose2d> m_poseSupplier;
  private final Supplier<LimelightData> m_limelightDataSupplier;
  private final DoubleSupplier m_turretAngleSupplier;

  private Pose2d m_pose;
  private LimelightData m_limelightData;
  private double m_turretAngle;
  private double m_updateTime;

  public TargetEstimator(Supplier<Pose2d> poseSupplier, Supplier<LimelightData> limelightDataSupplier, DoubleSupplier turretAngleSupplier) {
    m_poseSupplier = poseSupplier;
    m_limelightDataSupplier = limelightDataSupplier;
    m_turretAngleSupplier = turretAngleSupplier;
  }

  @Override
  public void periodic() {
    var limelightData = m_limelightDataSupplier.get();
    if (!limelightData.isTargetFound()) {
      return;
    }

    m_pose = m_poseSupplier.get();
    m_limelightData = limelightData;
    m_turretAngle = m_turretAngleSupplier.getAsDouble();
    m_updateTime = Timer.getFPGATimestamp();
  }

  public TargetEstimate getEstimate(Pose2d pose) {
    if (Timer.getFPGATimestamp() - m_updateTime > 5.0) {
      return null;
    }

    var movement = pose.relativeTo(m_pose);
    double dx = movement.getTranslation().getX();
    double dy = movement.getTranslation().getY();
    double dTheta = movement.getRotation().getDegrees();

    // TODO finish this

    return null;
  }
}
