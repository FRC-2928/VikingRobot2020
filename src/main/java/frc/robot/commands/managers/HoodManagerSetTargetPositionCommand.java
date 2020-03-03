package frc.robot.commands.managers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.managers.HoodPositionManager;

public class HoodManagerSetTargetPositionCommand extends RunCommand {
  public HoodManagerSetTargetPositionCommand(HoodPositionManager manager, double degrees) {
    this(manager, () -> degrees);
  } 

  public HoodManagerSetTargetPositionCommand(HoodPositionManager manager, DoubleSupplier positionSupplier) {
    super(() -> {
      manager.setTargetDegrees(positionSupplier.getAsDouble());
    }, manager);
  }
}
