package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;

public interface DriverOI {
    Button getShiftLowButton();

    Button getShiftHighButton();

    DoubleSupplier getMoveSupplier();

    DoubleSupplier getRotateSupplier();

    Button getAutoShootingButton();

    Button getFenderShotButton();

    Button getInitiationlineShotButton();

    Button getShooterDebugButton();

    Button getFeedButton();

    Button getSetpointShootingButton();

    Button getGroundIntakeButton();

    Button getStationIntakeButton();

    Button getClimbTrigger();
}