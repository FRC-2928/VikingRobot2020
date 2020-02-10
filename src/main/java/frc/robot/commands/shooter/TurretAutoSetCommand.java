package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.types.LimelightData;

public class TurretAutoSetCommand extends TurretSetPositionCommand {
  public TurretAutoSetCommand(TurretSubsystem turret, Supplier<LimelightData> limelightDataSupplier, DoubleSupplier gyroAngleSupplier) {
    super(turret, () -> getTargetPosition(turret, limelightDataSupplier, gyroAngleSupplier));
  }

  private static double getTargetPosition(TurretSubsystem turret, Supplier<LimelightData> limelightDataSupplier, DoubleSupplier gyroAngleSupplier) {
    double targetPosition = -gyroAngleSupplier.getAsDouble();
    var limelightData = limelightDataSupplier.get();
    if (limelightData.isTargetFound()) {
      targetPosition = turret.getMeasuredPosition() - limelightData.getHorizontalAngle();
    }
    return targetPosition;
  }
}
