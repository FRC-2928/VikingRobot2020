package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.drive.DrivetrainArcadeDriveCommand;
import frc.robot.commands.drive.DrivetrainTankDriveCommand;
import frc.robot.commands.drive.TransmissionSetHighGearCommand;
import frc.robot.commands.drive.TransmissionSetLowGearCommand;
import frc.robot.subsystems.Subsystems;

public class OIBinder {

    public static void bind(Subsystems subsystems, OIBindable oi) {
        oi.bindArcadeDrive(new DrivetrainArcadeDriveCommand(subsystems.drivetrain));
        oi.bindTankDrive(new DrivetrainTankDriveCommand(subsystems.drivetrain));

        oi.bindTransmissionSetHigh(new TransmissionSetHighGearCommand(subsystems.transmission));
        oi.bindTransmissionSetLow(new TransmissionSetLowGearCommand(subsystems.transmission));
        oi.bindTransmissionToggle(new ConditionalCommand(
            new TransmissionSetLowGearCommand(subsystems.transmission), 
            new TransmissionSetHighGearCommand(subsystems.transmission), 
            () -> subsystems.transmission.isHighGear())
        );
        
    }

}
