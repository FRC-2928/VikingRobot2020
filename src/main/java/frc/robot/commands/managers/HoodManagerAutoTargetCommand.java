package frc.robot.commands.managers;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.managers.HoodPositionManager;

public class HoodManagerAutoTargetCommand extends HoodManagerSetTargetPositionCommand {
    public HoodManagerAutoTargetCommand(HoodPositionManager manager, DoubleSupplier distanceSupplier) {
        super(manager, () -> getTargetDegrees(manager, distanceSupplier));
    }

    private static double getTargetDegrees(HoodPositionManager manager, DoubleSupplier distanceSupplier) {
        double distance = distanceSupplier.getAsDouble();
        if (distance < 0) {
            return manager.getTargetDegrees();
        }
        return 0.0;
    }
}
