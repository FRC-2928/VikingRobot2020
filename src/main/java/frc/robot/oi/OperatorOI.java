package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;

public interface OperatorOI {

    Button getEnableFeederButton();

    Button getDisableFeederButton();

    Button getReverseFeederButton();

    Button getEnableAutoTargetButton();

    Button getDisableAutoTargetButton();

    DoubleSupplier ClimberAdjustmentButton();

    Button spinColorWheelButton();

    Button turnToColorButton();

    Button enableFeederButton();

    Button disableFeederButton();

    Button reverseFeederButton();
}