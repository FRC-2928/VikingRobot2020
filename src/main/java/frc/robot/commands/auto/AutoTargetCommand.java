package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.FlywheelSetVelocityCommand;
import frc.robot.commands.shooter.HoodSetPositionCommand;
import frc.robot.commands.shooter.TurretSetPositionCommand;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.utilities.LimelightUtility.LimelightData;

public class AutoTargetCommand extends ScheduleCommand {
  public AutoTargetCommand(HoodSubsystem hood, TurretSubsystem turret, FlywheelSubsystem flywheel,
      Supplier<LimelightData> limeliteDataSupplier, DoubleSupplier gyroAngleSupplier) {
    this(hood, turret, flywheel, limeliteDataSupplier, gyroAngleSupplier, new Trigger(() -> true));
  }

  public AutoTargetCommand(HoodSubsystem hood, TurretSubsystem turret, FlywheelSubsystem flywheel,
      Supplier<LimelightData> limeliteDataSupplier, DoubleSupplier gyroAngleSupplier, Trigger flywheelTrigger) {
    super(
      new HoodSetPositionCommand(hood, () -> {
        return 0.0;
      }),
      new TurretSetPositionCommand(turret, () -> {
        return 0.0;
      }),
      new FlywheelSetVelocityCommand(flywheel, () -> {
        return 0.0;
      }, flywheelTrigger)
    );
  }
}
