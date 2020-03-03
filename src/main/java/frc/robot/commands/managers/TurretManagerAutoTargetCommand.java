package frc.robot.commands.managers;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.managers.TurretPositionManager;

public class TurretManagerAutoTargetCommand extends TurretManagerSetTargetPositionCommand {
    public TurretManagerAutoTargetCommand(TurretPositionManager manager, DoubleSupplier distanceSupplier) {
        super(manager, () -> getTargetDegrees(manager, distanceSupplier));
    }

    private static double getTargetDegrees(TurretPositionManager manager, DoubleSupplier distanceSupplier) {
        double distance = distanceSupplier.getAsDouble();
        if (distance < 0) {
            return manager.getTargetDegrees();
        }
        return 0.0;
    }
}
