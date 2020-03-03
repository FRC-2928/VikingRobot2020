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

    // ---------------- Intake ----------------------------

    @Override
    public Button getGroundIntakeButton() {
        return new Button(() -> m_controller.getTriggerAxis(Hand.kLeft) > 0.1);
    }

    @Override
    public Button getStationIntakeButton() {
        return new JoystickButton(m_controller, XboxController.Button.kBumperLeft.value);
    }

    // ---------------- Climber ----------------------------

    @Override
    public Button getClimbTrigger() {
        return new JoystickButton(m_controller, XboxController.Axis.kRightTrigger.value);
    }

    // ---------------- Shooting ----------------------------

    @Override
    public Button getAutoShootingButton() {
        return new JoystickButton(m_controller, XboxController.Button.kBumperRight.value);
    }

    @Override
    public Button getSetpointShootingButton(){
        return new Button(() -> m_controller.getTriggerAxis(Hand.kRight) > 0.1);
    }

    @Override
    public Button getFenderShotButton() {
        return new Button(() -> m_controller.getPOV() == 180);
    }

    @Override
    public Button getInitiationlineShotButton() {
        return new Button(() -> m_controller.getPOV() == 0);
    }

    @Override
    public Button getShooterDebugButton() {
        return new Button(() -> m_controller.getPOV() == 90);
    }

    @Override
    public Button getFeedButton() {
        return new Button(() -> m_controller.getTriggerAxis(Hand.kRight) > 0.1);
    }

    // ---------------- Drivetrain ----------------------------

    @Override
    public Button getShiftLowButton() {
        return new JoystickButton(m_controller, XboxController.Button.kX.value);
    }

    @Override
    public Button getShiftHighButton() {
        return new JoystickButton(m_controller, XboxController.Button.kY.value);
    }

    @Override
    public DoubleSupplier getMoveSupplier() {
        return () -> -m_controller.getY(Hand.kLeft)*0.8;
    }

    @Override
    public DoubleSupplier getRotateSupplier() {
        return () -> m_controller.getX(Hand.kRight)*0.8;
    }
}