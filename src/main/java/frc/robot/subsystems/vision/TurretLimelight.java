package frc.robot.subsystems.vision;

import org.ballardrobotics.subsystems.vision.LimelightBase;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.TurretLimelightConstants;

public class TurretLimelight extends LimelightBase {
  private double m_distance;

  public TurretLimelight() {
    super(NetworkTableInstance.getDefault().getTable(TurretLimelightConstants.kTableName));
  }

  @Override
  public void periodic() {
    super.periodic();

    if (m_targetFound) {
      // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
      double h1 = TurretLimelightConstants.kMountHeightMeters;
      double h2 = TurretLimelightConstants.kTargetHeightMeters;
      double a1 = TurretLimelightConstants.kMountAngleDegrees;
      double a2 = m_verticalAngle;
      m_distance = (h2 - h1) / Math.tan(Math.toRadians(a1 + a2));
    } else {
      m_distance = -1.0;
    }
  }

  @Override
  public void setCameraMode(CameraMode mode) {
    super.setCameraMode(mode);

    if (mode == CameraMode.Tracking) {
      setPipeline(TurretLimelightConstants.kDrivePipeline);
    } else {
      setPipeline(TurretLimelightConstants.kTrackingPipeline);
    }
  }

  public double getDistance() {
    return m_distance;
  }
}
