package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.Button;

public interface OperatorOI {

    Button deployToTop(); //no longer just for testing

    DoubleSupplier adjustClimber(); 

    Button getEnableAutoTargetButton();

    Button getDisableAutoTargetButton();
    
    DoubleSupplier moveTurretSupplier();

    Button getMoveTurretButton();

    Button turnWheelButton();

    Button turnToColorButton();
    
    Button turnWheelThreeTimes();

    Button enableFeederButton();

    Button disableFeederButton();

    Button reverseFeederButton();

    Button feedButton();

    Button turnTurretToWheel();

    Button incrementShootFromButton();

    Button decrementShootFromButton();

    Button getShootFromWallButton();

    Button getShootFromLineButton();

    Button getShootFromTrenchButton();

    // Button shootFromTrenchButton();
    
}