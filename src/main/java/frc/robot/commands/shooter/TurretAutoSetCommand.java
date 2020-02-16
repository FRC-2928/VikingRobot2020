package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.types.LimelightData;
import frc.robot.types.TargetEstimate;

public class TurretAutoSetCommand extends TurretSetPositionCommand {
  public TurretAutoSetCommand(TurretSubsystem turret, Supplier<Pose2d> poseSupplier, Supplier<LimelightData> limelightDataSupplier, Function<Pose2d, TargetEstimate> estimateFunc) {
    super(turret, () -> getTargetPosition(turret, poseSupplier, limelightDataSupplier, estimateFunc));
  }

  private static double getTargetPosition(TurretSubsystem turret, Supplier<Pose2d> poseSupplier, Supplier<LimelightData> limelightDataSupplier, Function<Pose2d, TargetEstimate> estimateFunc) {
    var pose = poseSupplier.get();
    var limelightData = limelightDataSupplier.get();
    var targetEstimate = estimateFunc.apply(pose);

    if (limelightData.isTargetFound()) {
      return turret.getMeasuredPosition() - limelightData.getHorizontalAngle();
    }

    if (targetEstimate != null) {
      return targetEstimate.getAngle();
    }

    return -pose.getRotation().getDegrees();
  }
}
