package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.types.LimelightData;

public class FlywheelAutoSetCommand extends FlywheelSetVelocityCommand {
  public FlywheelAutoSetCommand(FlywheelSubsystem flywheel, Supplier<LimelightData> limelightDataSupplier, Trigger trigger) {
    super(flywheel, () -> getTargetVelocity(limelightDataSupplier), trigger);
  }

  private static double getTargetVelocity(Supplier<LimelightData> limelightDataSupplier) {
    return 0.0;
  }
}
