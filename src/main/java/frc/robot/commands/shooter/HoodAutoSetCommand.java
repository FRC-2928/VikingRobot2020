package frc.robot.commands.shooter;

import java.util.function.Supplier;

import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.types.LimelightData;

public class HoodAutoSetCommand extends HoodSetPositionCommand {
  public HoodAutoSetCommand(HoodSubsystem hood, Supplier<LimelightData> limelightDataSupplier) {
    super(hood, () -> getTargetPosition(limelightDataSupplier));
  }

  private static double getTargetPosition(Supplier<LimelightData> limelightDataSupplier) {
    return 0.0;
  }
}
