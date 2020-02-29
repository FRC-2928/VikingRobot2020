package frc.robot.subsystems;

import frc.robot.commands.climber.ElevatorStopCommand;
import frc.robot.commands.drive.DrivetrainStopCommand;
import frc.robot.commands.drive.TransmissionSetHighGearCommand;
import frc.robot.commands.indexer.FeederStopCommand;
import frc.robot.commands.indexer.HopperStopCommand;
import frc.robot.commands.shooter.FlywheelStopCommand;
import frc.robot.commands.shooter.HoodStopCommand;
import frc.robot.commands.shooter.TurretStopCommand;
import frc.robot.subsystems.climber.ElevatorSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.TransmissionSubsystem;
import frc.robot.subsystems.indexer.FeederSubsystem;
import frc.robot.subsystems.indexer.HopperSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;

/**
 * Subsystems acts as a container for all subsystems.
 */
public class Subsystems {
    public final DrivetrainSubsystem drivetrain;
    public final TransmissionSubsystem transmission;
    public final ElevatorSubsystem elevator;
    public final FlywheelSubsystem flywheel;
    public final HoodSubsystem hood;
    public final TurretSubsystem turret;
    public final FeederSubsystem feeder;
    public final HopperSubsystem hopper;

    public Subsystems() {
        drivetrain = DrivetrainSubsystem.create();
        transmission = TransmissionSubsystem.create();
        elevator = ElevatorSubsystem.create();
        flywheel = FlywheelSubsystem.create();
        hood = HoodSubsystem.create();
        turret = TurretSubsystem.create();
        feeder = FeederSubsystem.create();
        hopper = HopperSubsystem.create();

        setDefaultCommands();
    }

    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(new DrivetrainStopCommand(drivetrain));
        transmission.setDefaultCommand(new TransmissionSetHighGearCommand(transmission));
        elevator.setDefaultCommand(new ElevatorStopCommand(elevator));
        flywheel.setDefaultCommand(new FlywheelStopCommand(flywheel));
        hood.setDefaultCommand(new HoodStopCommand(hood));
        turret.setDefaultCommand(new TurretStopCommand(turret));
        feeder.setDefaultCommand(new FeederStopCommand(feeder));
        hopper.setDefaultCommand(new HopperStopCommand(hopper));
    }
}
