package frc.robot.commands.managers;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.managers.FlywheelVelocityManager;

public class FlywheelManagerAutoTargetCommand extends FlywheelManagerSetTargetVelocityCommand {
  public FlywheelManagerAutoTargetCommand(FlywheelVelocityManager manager, DoubleSupplier distanceSupplier) {
    super(manager, () -> getTargetRPM(distanceSupplier));
  }

  private static double getTargetRPM(DoubleSupplier distanceSupplier) {
    double distance = distanceSupplier.getAsDouble();
    if (distance < 0) {
      return 0;
    }
    return 0.0;
  }
}
