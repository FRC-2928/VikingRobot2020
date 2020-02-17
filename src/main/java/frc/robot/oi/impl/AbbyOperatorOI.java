package frc.robot.oi.impl;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.oi.OperatorOI;

public class AbbyOperatorOI implements OperatorOI {
    private XboxController m_controller;

    public AbbyOperatorOI(XboxController controller) {
        m_controller = controller;
    }
   
    // --------------- Climber ------------------

    @Override
    public Button deployClimberHigh() {
        return new Button(() -> {
            return m_controller.getPOV() == Constants.OIConstants.kUp;
        });
    }

    @Override
    public Button deployClimberMid() {
        return new Button(() -> {
            return m_controller.getPOV() == Constants.OIConstants.kMid;
        });
       
    }

    @Override
    public Button deployClimberMidTwo() {
        return new Button(() -> {
            return m_controller.getPOV() == Constants.OIConstants.kMidTwo;
        });
    }

    @Override
    public Button deployClimberLow() {
        return new Button(() -> {
            return m_controller.getPOV() == Constants.OIConstants.kDown;
        });
    }

    @Override
    public Button deployToTop(){
        return new JoystickButton(m_controller, XboxController.Button.kX.value);
    }
  
    @Override
    public DoubleSupplier climberAdjustmentButton() {
        return() -> -m_controller.getY(Hand.kLeft);
    }

    // ------------ Auto Targeting ------------------

    @Override
    public Button getEnableAutoTargetButton() {
        return new JoystickButton(m_controller, XboxController.Button.kStart.value);
    }

    @Override
    public Button getDisableAutoTargetButton() {
        return new JoystickButton(m_controller, XboxController.Button.kBack.value);
    }

    // ------------ Shooting ---------------------
    @Override
    public Button incrementShootFromButton() {
        return new JoystickButton(m_controller, XboxController.Button.kBumperRight.value);
    }

    @Override
    public Button decrementShootFromButton() {
        return new JoystickButton(m_controller, XboxController.Button.kBumperLeft.value);
    }
    // ------------ Color Wheel ------------------

    @Override
    public Button turnWheelButton() {
        return new Button(() -> {
            return m_controller.getTriggerAxis(Hand.kLeft) > 0.1 && 
                   m_controller.getTriggerAxis(Hand.kRight) > 0.1;
        });
    }

    @Override
    public Button turnToColorButton() {
        return new JoystickButton(m_controller, XboxController.Axis.kLeftTrigger.value);
    }

    @Override
    public Button turnWheelThreeTimes() {
        return new JoystickButton(m_controller, XboxController.Axis.kRightTrigger.value);
    }

    @Override
    public Button turnTurretToWheel() {
        return new JoystickButton(m_controller, XboxController.Button.kStickRight.value);
    }

    // ------------ Feeder ------------------

    @Override
    public Button enableFeederButton() {
        return new JoystickButton(m_controller, XboxController.Button.kA.value);
    }

    @Override
    public Button disableFeederButton() {
        return new JoystickButton(m_controller, XboxController.Button.kY.value);
    }

    @Override
    public Button reverseFeederButton() {
        return new JoystickButton(m_controller, XboxController.Button.kB.value);
    }
}