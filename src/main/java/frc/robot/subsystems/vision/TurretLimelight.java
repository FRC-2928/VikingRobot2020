package frc.robot.subsystems.vision;

import org.ballardrobotics.subsystems.vision.LimelightBase;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.TurretLimelightConstants;
import frc.robot.commands.vision.LimelightSetDriverModeCommand;
import frc.robot.commands.vision.LimelightSetTrackingModeCommand;

public class TurretLimelight extends LimelightBase {
  private double m_distance;
  private NetworkTableEntry m_skewFactorEntry;

  public TurretLimelight() {
    super(NetworkTableInstance.getDefault().getTable(TurretLimelightConstants.kTableName));
  }

  public void configureShuffleboard(ShuffleboardLayout stateLayout, ShuffleboardLayout controlLayout) {
    stateLayout.addBoolean("found", () -> m_data.isTargetFound());
    stateLayout.addNumber("xAngle", () -> m_data.getHorizontalAngle());
    stateLayout.addNumber("yAngle", () -> m_data.getVerticalAngle());
    stateLayout.addNumber("skew", () -> m_data.getSkew());
    stateLayout.addNumber("distance", this::getDistanceEstimate);
    stateLayout.addNumber("centerTargetAngle", this::getAngleToCenterTarget);
    
    m_skewFactorEntry = controlLayout.add("skewFactor", 0).getEntry();
    controlLayout.add("enable", new LimelightSetTrackingModeCommand(this));
    controlLayout.add("disable", new LimelightSetDriverModeCommand(this));
  }

  @Override
  public void periodic() {
    super.periodic();

    if (m_data.isTargetFound()) {
      // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
      double h1 = TurretLimelightConstants.kMountHeightMeters;
      double h2 = TurretLimelightConstants.kTargetHeightMeters;
      double a1 = TurretLimelightConstants.kMountAngleDegrees;
      double a2 = m_data.getVerticalAngle();
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

  public double getDistanceEstimate() {
    return m_distance;
  }

  public double getAngleToCenterTarget() {
    double skewFactor = 0.0;
    if (m_skewFactorEntry != null) {
      skewFactor = m_skewFactorEntry.getDouble(0.0);
    }
    return m_data.getHorizontalAngle() + m_data.getSkew() * skewFactor;
  }
}
