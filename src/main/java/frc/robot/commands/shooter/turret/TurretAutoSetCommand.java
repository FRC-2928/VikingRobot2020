package frc.robot.commands.shooter.turret;

import java.util.function.Supplier;

import org.ballardrobotics.utilities.TurretUtility;

import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem.TrackingType;

public class TurretAutoSetCommand extends TurretSetPositionCommand {
  public class SensorState {
    public boolean visionFound;
    public double visionAngle;
    public boolean estimateFresh;
    public double estimateAngle;
    public double drivetrainHeading;
  }

  private Supplier<SensorState> m_sensorStateSupplier;
  private double m_correctionPosition;

  public TurretAutoSetCommand(TurretSubsystem turret, Supplier<SensorState> supplier) {
    super(turret, 0.0);
    m_sensorStateSupplier = supplier;
    m_positionSupplier = this::getTargetAngle;
  }

  @Override
  public void initialize() {
    m_turret.setTrackingType(TrackingType.None);
  }

  private double getTargetAngle() {
    var sensorState = m_sensorStateSupplier.get();
    double currentPosition = m_turret.getMeasuredPosition();
    
    if (m_turret.getTrackingType() == TrackingType.Correction) {
      double distanceFromCorrection = Math.abs(currentPosition - m_correctionPosition);
      if (distanceFromCorrection < 135.0 && sensorState.visionFound) {
        m_turret.setTrackingType(TrackingType.Vision);
      }
      else if (distanceFromCorrection < 135.0 && sensorState.estimateFresh) {
        m_turret.setTrackingType(TrackingType.Estimate);
      }
      else if (distanceFromCorrection < 10.0) {
        m_turret.setTrackingType(TrackingType.FaceForward);
      }
      return m_correctionPosition;
    }

    double targetPosition = currentPosition;
    if (sensorState.visionFound) {
      m_turret.setTrackingType(TrackingType.Vision);
      targetPosition = currentPosition + sensorState.visionAngle;
    }
    else if (sensorState.estimateFresh) {
      m_turret.setTrackingType(TrackingType.Estimate);
      targetPosition = currentPosition + sensorState.visionAngle;
    }
    else {
      m_turret.setTrackingType(TrackingType.FaceForward);
      double filteredHeading = sensorState.drivetrainHeading % 360.0;
      if (filteredHeading < 0.0) {
        filteredHeading += 360.0;
      }
      targetPosition = filteredHeading - 180.0;
    }

    if (!TurretUtility.angleInRange(targetPosition, TurretConstants.kMinAngle, TurretConstants.kMaxAngle)) {
      m_turret.setTrackingType(TrackingType.Correction);
      targetPosition = TurretUtility.getTargetAngle(targetPosition, currentPosition, TurretConstants.kMinAngle, TurretConstants.kMaxAngle);
      m_correctionPosition = targetPosition;
    }

    return targetPosition;
  }
}
