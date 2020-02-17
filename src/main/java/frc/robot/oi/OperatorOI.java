package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;

public interface OperatorOI {

    Button deployClimberHigh();

    Button deployClimberMid();

    Button deployClimberMidTwo();

    Button deployClimberLow();

    Button deployToTop(); //this is for testing

    DoubleSupplier climberAdjustmentButton();

    Button getEnableAutoTargetButton();

    Button getDisableAutoTargetButton();

    Button spinColorWheelButton();

    Button turnToColorButton();
    
    Button turnWheelThreeTimes();

    Button enableFeederButton();

    Button disableFeederButton();

    Button reverseFeederButton();

    Button turnTurretToWheel();

    Button rotateTurret();

    Button shootFromWallButton();

    Button shootFromLineButton();

    Button shootFromWheelButton();

    Button shootFromTrenchButton();
    
}