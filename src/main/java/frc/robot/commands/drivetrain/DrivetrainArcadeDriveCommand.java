package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class DrivetrainArcadeDriveCommand extends RunCommand {
  public DrivetrainArcadeDriveCommand(DrivetrainSubsystem drivetrain, DoubleSupplier throttle, DoubleSupplier rotate) {
    super(
      () -> {
        drivetrain.drive(throttle.getAsDouble(), rotate.getAsDouble());
      }, 
      drivetrain
    );
  }
}
