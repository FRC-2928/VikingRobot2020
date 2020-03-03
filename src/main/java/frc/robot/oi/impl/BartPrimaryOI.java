package frc.robot.oi.impl;

import java.util.function.Supplier;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.ArcadeDriveValue;
import org.ballardrobotics.types.supplied.PercentOutputValue;
import org.ballardrobotics.types.supplied.TankDriveValue;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.OIBindable;

public class BartPrimaryOI implements OIBindable {
    private XboxController m_controller;

    public BartPrimaryOI(XboxController controller) {
        m_controller = controller;
    }

    @Override
    public void bindArcadeDrive(SuppliedCommand<ArcadeDriveValue> command) {
        final double kMoveDeadband = 0.05;
        final double kRotateDeadband = 0.15;
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
            if (command.isScheduled()) {
                return false;
            }
            var arcadeDriveValue = arcadeDriveSupplier.get();
            boolean aboveDeadband = Math.abs(arcadeDriveValue.move) > kMoveDeadband || 
                                    Math.abs(arcadeDriveValue.rotate) > kRotateDeadband;
            return aboveDeadband;
        }).whenPressed(command);
    }

    @Override
    public void bindTankDrive(SuppliedCommand<TankDriveValue> command) {}

    @Override
    public void bindTransmissionSetLow(Command command) {}

    @Override
    public void bindTransmissionSetHigh(Command command) {}

    @Override
    public void bindTransmissionToggle(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kStickLeft.value).whenPressed(command);
    }

    @Override
    public void bindArmsStow(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kBumperLeft.value).whenReleased(command);
    }

    @Override
    public void bindArmsLower(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kBumperLeft.value).whenPressed(command);
    }

    @Override
    public void bindIntakeForwardRoller(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kBumperLeft.value).whenPressed(command);
    }

    @Override
    public void bindIntakeReverseRoller(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kX.value).whenPressed(command);
    }

    @Override
    public void bindIntakeStopRoller(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kBumperLeft.value).whenReleased(command);
        new JoystickButton(m_controller, XboxController.Button.kX.value).whenReleased(command);
    }

    @Override
    public void bindFlywheelOpenLoop(SuppliedCommand<PercentOutputValue> command) {}

    @Override
    public void bindFlywheelSetFromReference(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kBumperRight.value).whenPressed(command);
    }

    @Override
    public void bindFlywheelDisable(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kBumperRight.value).whenReleased(command);
    }

    @Override
    public void bindHoodOpenLoop(SuppliedCommand<PercentOutputValue> command) {}

    @Override
    public void bindHoodSetFromReference(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kBumperRight.value).whenPressed(command);
    }

    @Override
    public void bindHoodDisable(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kBumperRight.value).whenReleased(command);
    }

    @Override
    public void bindTurretOpenLoop(SuppliedCommand<PercentOutputValue> command) {
        final double kDeadband = 0.15;
        Supplier<PercentOutputValue> percentOutputSupplier = () -> {
            double value = m_controller.getX(Hand.kRight);
            if (Math.abs(value) < kDeadband) {
                value = 0.0;
            }
            return new PercentOutputValue(value);
        };
        command.setSupplier(percentOutputSupplier);

        new Button(() -> {
            if (command.isScheduled()) {
                return false;
            }
            var percentOutput = percentOutputSupplier.get();
            boolean aboveDeadband = Math.abs(percentOutput.value) > kDeadband;
            return aboveDeadband;
        }).whenPressed(command);
    }

    @Override
    public void bindTurretSetFromReference(Command command) {
        
    }

    @Override
    public void bindTurretDisable(Command command) {
        // TODO Auto-generated method stub

    }

    @Override
    public void bindHopperOpenLoop(SuppliedCommand<PercentOutputValue> command) {
        // TODO Auto-generated method stub

    }

    @Override
    public void bindHopperDisable(Command command) {}

    @Override
    public void bindFeederOpenLoop(SuppliedCommand<PercentOutputValue> command) {}

    @Override
    public void bindFeederDisable(Command command) {}

    @Override
    public void bindVisionFullEnable(Command command) {}

    @Override
    public void bindVisionTurretEnable(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kBumperRight.value).whenPressed(command);
    }

    @Override
    public void bindCancelAll(Command command) {
        // TODO Auto-generated method stub

    }

    @Override
    public void bindVisionFullDisable(Command command) {
        // TODO Auto-generated method stub

    }

    @Override
    public void bindVisionTurretDisable(Command command) {
        // TODO Auto-generated method stub

    }

    /*
    @Override
    public void bindArcadeDrive(SuppliedCommand<ArcadeDriveValue> command) {
        final double kMoveDeadband = 0.05;
        final double kRotateDeadband = 0.15;
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
    public void bindFlywheelSetFromReference(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kBumperRight.value).whenPressed(command);
    }

    @Override
    public void bindFlywheelDisable(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kBumperRight.value).whenReleased(command);
    }

    @Override
    public void bindTurretOpenLoop(SuppliedCommand<PercentOutputValue> command) {
        final double kDeadband = 0.15;
        Supplier<PercentOutputValue> percentOutputSupplier = () -> {
            double value = m_controller.getX(Hand.kRight);
            if (Math.abs(value) < kDeadband) {
                value = 0.0;
            }
            return new PercentOutputValue(value);
        };
        command.setSupplier(percentOutputSupplier);

        new Button(() -> {
            var percentOutput = percentOutputSupplier.get();
            boolean aboveDeadband = Math.abs(percentOutput.value) > kDeadband;
            return aboveDeadband && !command.isScheduled();
        }).whenPressed(command);
    }

    @Override
    public void bindTransmissionSetHigh(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kB.value).whenPressed(command);
    }

    @Override
    public void bindTransmissionSetLow(Command command) {
        new JoystickButton(m_controller, XboxController.Button.kA.value).whenPressed(command);
    }
    */
}
