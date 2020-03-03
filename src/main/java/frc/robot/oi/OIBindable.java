package frc.robot.oi;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.ArcadeDriveValue;
import org.ballardrobotics.types.supplied.PercentOutputValue;
import org.ballardrobotics.types.supplied.TankDriveValue;

import edu.wpi.first.wpilibj2.command.Command;

public interface OIBindable {
    // Drivetrain
    void bindArcadeDrive(SuppliedCommand<ArcadeDriveValue> command);
    void bindTankDrive(SuppliedCommand<TankDriveValue> command);

    // Transmission
    void bindTransmissionSetLow(Command command);
    void bindTransmissionSetHigh(Command command);
    void bindTransmissionToggle(Command command);

    // Arms
    void bindArmsStow(Command command);
    void bindArmsLower(Command command);

    // Intake Rollers
    void bindIntakeForwardRoller(Command command);
    void bindIntakeReverseRoller(Command command);
    void bindIntakeStopRoller(Command command);

    // Flywheel
    void bindFlywheelOpenLoop(SuppliedCommand<PercentOutputValue> command);
    void bindFlywheelSetFromReference(Command command);
    void bindFlywheelDisable(Command command);

    // Hood
    void bindHoodOpenLoop(SuppliedCommand<PercentOutputValue> command);
    void bindHoodSetFromReference(Command command);
    void bindHoodDisable(Command command);

    // Turret
    void bindTurretOpenLoop(SuppliedCommand<PercentOutputValue> command);
    void bindTurretSetFromReference(Command command);
    void bindTurretDisable(Command command);

    // Hopper
    void bindHopperOpenLoop(SuppliedCommand<PercentOutputValue> command);
    void bindHopperDisable(Command command);

    // Feeder
    void bindFeederOpenLoop(SuppliedCommand<PercentOutputValue> command);
    void bindFeederDisable(Command command);

    // Vision
    void bindVisionFullEnable(Command command);
    void bindVisionFullDisable(Command command);
    void bindVisionTurretEnable(Command command);
    void bindVisionTurretDisable(Command command);

    // Stop all active commands.
    void bindCancelAll(Command command);
}
