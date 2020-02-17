package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;

public interface OperatorOI {

    Button deployClimberHigh();

    Button deployClimberMid();

    Button deployClimberLow();

    DoubleSupplier climberAdjustmentButton();

    Button getEnableAutoTargetButton();

    Button getDisableAutoTargetButton();

    Button spinColorWheelButton();

    Button turnToColorButton();

    Button enableFeederButton();

    Button disableFeederButton();

    Button reverseFeederButton();
}