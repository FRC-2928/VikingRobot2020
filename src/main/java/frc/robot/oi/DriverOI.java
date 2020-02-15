package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;

public interface DriverOI {
    Button getShiftLowButton();

    Button getShiftHighButton();

    DoubleSupplier getMoveSupplier();

    DoubleSupplier getRotateSupplier();

    Button getAutoShootingButton();

    Button getGroundIntakeButton();

    Button getStationIntakeButton();

    Button getClimbTrigger();
}