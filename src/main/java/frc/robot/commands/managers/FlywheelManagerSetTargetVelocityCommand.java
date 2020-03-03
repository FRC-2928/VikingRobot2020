package frc.robot.commands.managers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.managers.FlywheelVelocityManager;

public class FlywheelManagerSetTargetVelocityCommand extends RunCommand {
  public FlywheelManagerSetTargetVelocityCommand(FlywheelVelocityManager manager, double rpm) {
    this(manager, () -> rpm);
  } 

  public FlywheelManagerSetTargetVelocityCommand(FlywheelVelocityManager manager, DoubleSupplier velocitySupplier) {
    super(() -> {
      manager.setTargetRPM(velocitySupplier.getAsDouble());
    }, manager);
  }
}
