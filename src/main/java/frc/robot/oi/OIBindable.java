package frc.robot.oi;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.ArcadeDriveValue;
import org.ballardrobotics.types.supplied.TankDriveValue;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Add your docs here.
 */
public interface OIBindable {
    // Drivetrain
    default void bindArcadeDrive(SuppliedCommand<ArcadeDriveValue> command) {}
    default void bindTankDrive(SuppliedCommand<TankDriveValue> command) {}

    // Transmission
    default void bindTransmissionSetLow(Command command) {}
    default void bindTransmissionSetHigh(Command command) {}
    default void bindTransmissionToggle(Command command) {}

    // Flywheel
    default void bindFlywheelEnable(Command command) {}
    default void bindFlywheelDisable(Command command) {}

    // Intake Arms
    default void bindIntakeStow(Command command) {}
    default void bindIntakeLoadingBay(Command command) {}
    default void bindIntakeFloorPickup(Command command) {}

    // Intake Rollers
    default void bindIntakeForwardRoller(Command command) {}
    default void bindIntakeReverseRoller(Command command) {}
    default void bindIntakeStopRoller(Command command) {}

    // Shooter
    default void bindShooterEnableAutoTarget(Command command) {}
    default void bindShooterDisableAutoTarget(Command command) {}
    default void bindShooterInitLineSetpoint(Command command) {}
    default void bindShooterFrontTrenchSetpoint(Command command) {}
    default void bindShooterFarTrenchSetpoint(Command command) {}
    default void bindShooterCenterFieldSetpoint(Command command) {}
}
