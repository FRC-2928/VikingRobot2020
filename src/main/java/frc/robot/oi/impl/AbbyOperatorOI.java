package frc.robot.oi.impl;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.OperatorOI;

public class AbbyOperatorOI implements OperatorOI {
    private XboxController m_controller;

    public AbbyOperatorOI(XboxController controller) {
        m_controller = controller;
    }
   
    @Override
    public Button getEnableFeederButton() {
        return new JoystickButton(m_controller, XboxController.Button.kA.value);
    }

    @Override
    public Button getDisableFeederButton() {
        return new JoystickButton(m_controller, XboxController.Button.kB.value);
    }

    @Override
    public Button getReverseFeederButton() {
        return new JoystickButton(m_controller, XboxController.Button.kX.value);
    }

    @Override
    public Button getEnableAutoTargetButton() {
        return new JoystickButton(m_controller, XboxController.Button.kStart.value);
    }

    @Override
    public Button getDisableAutoTargetButton() {
        return new JoystickButton(m_controller, XboxController.Button.kBack.value);
    }

    @Override
    public DoubleSupplier ClimberAdjustmentButton() {
        return() -> -m_controller.getY(Hand.kLeft);
    }

    @Override
    public Button spinColorWheelButton() {
        return new JoystickButton(m_controller, XboxController.Axis.kLeftTrigger.value);
    }

    @Override
    public Button turnToColorButton() {
        return new JoystickButton(m_controller, XboxController.Button.kBumperLeft.value);
    }

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