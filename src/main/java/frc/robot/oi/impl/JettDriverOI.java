package frc.robot.oi.impl;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.DriverOI;

public class JettDriverOI implements DriverOI {
    private XboxController m_controller;

    public JettDriverOI(XboxController controller) {
        m_controller = controller;
    }

    @Override
    public Button getGroundIntakeButton() {
        return new JoystickButton(m_controller, XboxController.Axis.kLeftTrigger.value);
    }

    @Override
    public Button getStationIntakeButton() {
        return new JoystickButton(m_controller, XboxController.Button.kBumperLeft.value);
    }

    @Override
    public Button getClimbTrigger() {
        return new JoystickButton(m_controller, XboxController.Axis.kRightTrigger.value);
    }

    @Override
    public Button getAutoShootingButton() {
        return new JoystickButton(m_controller, XboxController.Button.kBumperRight.value);
    }

    @Override
    public Button getShiftLowButton() {
        return new JoystickButton(m_controller, XboxController.Button.kStickLeft.value);
    }

    @Override
    public Button getShiftHighButton() {
        return new JoystickButton(m_controller, XboxController.Button.kStickRight.value);
    }

    @Override
    public DoubleSupplier getMoveSupplier() {
        return () -> -m_controller.getY(Hand.kLeft);
    }

    @Override
    public DoubleSupplier getRotateSupplier() {
        return () -> m_controller.getX(Hand.kLeft);
    }
}