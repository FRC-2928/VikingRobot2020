package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;

public interface OperatorOI {

    Button getEnableFeederButton();

    Button getDisableFeederButton();

    Button getReverseFeederButton();

    // Button getEnableAutoTargetButton();

    // Button getDisableAutoTargetButton();

    // Button getDeployClimberButton();
}