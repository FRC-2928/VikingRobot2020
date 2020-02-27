package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;

public interface OperatorOI {

    DoubleSupplier deployToTop(); //no longer just for testing

    DoubleSupplier lowerClimber(); 

    Button getEnableAutoTargetButton();

    Button getDisableAutoTargetButton();

    Button turnWheelButton();

    Button turnToColorButton();
    
    Button turnWheelThreeTimes();

    Button enableFeederButton();

    Button disableFeederButton();

    Button reverseFeederButton();

    Button turnTurretToWheel();

    Button incrementShootFromButton();

    Button decrementShootFromButton();

    // Button shootFromWallButton();

    // Button shootFromLineButton();

    // Button shootFromWheelButton();

    // Button shootFromTrenchButton();
    
}