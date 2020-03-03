package frc.robot.commands.managers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.managers.TurretPositionManager;

public class TurretManagerSetTargetPositionCommand extends RunCommand {
  public TurretManagerSetTargetPositionCommand(TurretPositionManager manager, double degrees) {
    this(manager, () -> degrees);
  } 

  public TurretManagerSetTargetPositionCommand(TurretPositionManager manager, DoubleSupplier positionSupplier) {
    super(() -> {
      manager.setTargetDegrees(positionSupplier.getAsDouble());
    }, manager);
  }
}