package frc.robot.oi.impl;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.commands.climber.DeployClimber;
import frc.robot.oi.OperatorOI;

public class AbbiOperatorOI implements OperatorOI {
    private XboxController m_controller;

    public AbbiOperatorOI(XboxController controller) {
        m_controller = controller;
    }
   
    // --------------- Climber ------------------

    @Override
    public Button deployToTop(){
        return new Button(() -> m_controller.getPOV() != -1);
    }

    @Override 
    public DoubleSupplier adjustClimber(){
        return() -> (m_controller.getY(Hand.kLeft));
    }

    // ------------ Auto Targeting ------------------

    @Override
    public Button getEnableAutoTargetButton() {
        return new JoystickButton(m_controller, XboxController.Button.kBack.value);
    }

    @Override
    public Button getDisableAutoTargetButton() {
        return new JoystickButton(m_controller, XboxController.Button.kStart.value);
    }

    // ------------ Turret -----------------------
    
    @Override
    public DoubleSupplier moveTurretSupplier(){
        return() -> -(0.6)*m_controller.getX(Hand.kRight);
    }

    @Override
    public Button getMoveTurretButton() {
        return new Button(() -> Math.abs(m_controller.getX(Hand.kRight)) > 0.15);
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

    @Override
    public Button getShootFromWallButton(){
        return new Button(() -> m_controller.getPOV() == 180);
    }

    @Override
    public Button getShootFromLineButton() {
        return new Button(() -> m_controller.getPOV() == 270);
    }

    @Override
    public Button getShootFromTrenchButton() {
        return new Button(() -> m_controller.getPOV() == 360);
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

    @Override
    public Button feedButton() {
        return new JoystickButton(m_controller, XboxController.Button.kX.value);
    }
}