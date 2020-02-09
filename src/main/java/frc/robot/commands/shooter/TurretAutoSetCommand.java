package frc.robot.commands.shooter;

import java.util.function.Supplier;

import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.types.LimelightData;

public class TurretAutoSetCommand extends TurretSetPositionCommand {
  public TurretAutoSetCommand(TurretSubsystem turret, Supplier<LimelightData> limelightDataSupplier) {
    super(turret, () -> getTargetPosition(limelightDataSupplier));
  }

  private static double getTargetPosition(Supplier<LimelightData> limelightDataSupplier) {
    return 0.0;
  }
}
