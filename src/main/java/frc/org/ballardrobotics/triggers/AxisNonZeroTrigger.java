package frc.org.ballardrobotics.triggers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AxisNonZeroTrigger extends Trigger {

    public AxisNonZeroTrigger(DoubleSupplier axisValueSupplier) {
        this(axisValueSupplier, 0.2);
    }

    public AxisNonZeroTrigger(DoubleSupplier axisValueSupplier, double deadband) {
        super(() -> Math.abs(axisValueSupplier.getAsDouble()) > deadband);
    }

}
