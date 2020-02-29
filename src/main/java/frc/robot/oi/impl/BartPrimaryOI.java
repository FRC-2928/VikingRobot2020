package frc.robot.oi.impl;

import java.util.function.Supplier;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.ArcadeDriveValue;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.OIBindable;

public class BartPrimaryOI implements OIBindable {
    private static final double kMoveDeadband = 0.05;
    private static final double kRotateDeadband = 0.15;

    private XboxController m_controller;

    public BartPrimaryOI(XboxController controller) {
        m_controller = controller;
    }

    @Override
    public void bindArcadeDrive(SuppliedCommand<ArcadeDriveValue> command) {
        Supplier<ArcadeDriveValue> arcadeDriveSupplier = () -> {
            double move = m_controller.getTriggerAxis(Hand.kRight) - m_controller.getTriggerAxis(Hand.kLeft);
            if (Math.abs(move) < kMoveDeadband) {
                move = 0.0;
            }
            double rotate = m_controller.getX(Hand.kLeft);
            if (Math.abs(rotate) < kRotateDeadband) {
                rotate = 0.0;
            }
            return new ArcadeDriveValue(move, rotate);
        };
        command.setSupplier(arcadeDriveSupplier);

        new Button(() -> {
            var arcadeDriveValue = arcadeDriveSupplier.get();
            boolean aboveDeadband = Math.abs(arcadeDriveValue.move) > kMoveDeadband || 
                                    Math.abs(arcadeDriveValue.rotate) > kRotateDeadband;
            return aboveDeadband && !command.isScheduled();
        }).whenPressed(command);
    }

    @Override
    public void bindTransmissionToggle(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kStickLeft.value).whenPressed(command);
    }

    @Override
    public void bindFlywheelEnable(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kBumperRight.value).whenPressed(command);
    }

    @Override
    public void bindFlywheelDisable(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kBumperRight.value).whenReleased(command);
    }
}
