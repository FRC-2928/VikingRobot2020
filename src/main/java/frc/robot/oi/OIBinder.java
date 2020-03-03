package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.commands.chassis.drivetrain.DrivetrainArcadeDriveCommand;
import frc.robot.commands.chassis.drivetrain.DrivetrainTankDriveCommand;
import frc.robot.commands.chassis.transmission.TransmissionSetHighGearCommand;
import frc.robot.commands.chassis.transmission.TransmissionSetLowGearCommand;
import frc.robot.commands.indexer.IndexerAutoFeedCommand;
import frc.robot.commands.indexer.IndexerAutoIndexCommand;
import frc.robot.commands.indexer.IndexerStopCommand;
import frc.robot.commands.indexer.feeder.FeederSetPercentOutputCommand;
import frc.robot.commands.indexer.feeder.FeederStopCommand;
import frc.robot.commands.indexer.hopper.HopperSetPercentOutputCommand;
import frc.robot.commands.indexer.hopper.HopperStopCommand;
import frc.robot.commands.managers.FlywheelManagerAutoTargetCommand;
import frc.robot.commands.managers.FlywheelManagerSetTargetVelocityCommand;
import frc.robot.commands.managers.HoodManagerAutoTargetCommand;
import frc.robot.commands.managers.HoodManagerSetTargetPositionCommand;
import frc.robot.commands.managers.TurretManagerAutoTargetCommand;
import frc.robot.commands.managers.TurretManagerSetTargetPositionCommand;
import frc.robot.commands.shooter.flywheel.FlywheelSetPercentOutputCommand;
import frc.robot.commands.shooter.flywheel.FlywheelSetVelocityCommand;
import frc.robot.commands.shooter.flywheel.FlywheelStopCommand;
import frc.robot.commands.shooter.hood.HoodSetPercentOutputCommand;
import frc.robot.commands.shooter.hood.HoodSetPositionCommand;
import frc.robot.commands.shooter.hood.HoodStopCommand;
import frc.robot.commands.shooter.turret.TurretSetPercentOutputCommand;
import frc.robot.commands.shooter.turret.TurretSetPositionCommand;
import frc.robot.commands.shooter.turret.TurretStopCommand;
import frc.robot.commands.vision.LimelightSetDriverModeCommand;
import frc.robot.commands.vision.LimelightSetTrackingModeCommand;
import frc.robot.subsystems.Subsystems;

public class OIBinder {
    /*
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

        // TODO Bind Arms and Rollers
        
        oi.bindFlywheelOpenLoop(new FlywheelSetPercentOutputCommand(subsystems.flywheel));
        oi.bindFlywheelSetFromReference(new FlywheelSetVelocityCommand(subsystems.flywheel, subsystems.managers.flywheelVelocityManager::getTargetRPM));
        oi.bindFlywheelDisable(new FlywheelStopCommand(subsystems.flywheel));

        oi.bindHoodOpenLoop(new HoodSetPercentOutputCommand(subsystems.hood));
        oi.bindHoodSetFromReference(new HoodSetPositionCommand(subsystems.hood, subsystems.managers.hoodPositionManager::getTargetDegrees));
        oi.bindHoodDisable(new HoodStopCommand(subsystems.hood));

        oi.bindTurretOpenLoop(new TurretSetPercentOutputCommand(subsystems.turret));
        oi.bindTurretSetFromReference(new TurretSetPositionCommand(subsystems.turret, subsystems.managers.turretPositionManager::getTargetDegrees));
        oi.bindTurretDisable(new TurretStopCommand(subsystems.turret));

        oi.bindHopperOpenLoop(new HopperSetPercentOutputCommand(subsystems.hopper));
        oi.bindHopperDisable(new HopperStopCommand(subsystems.hopper));

        oi.bindFeederOpenLoop(new FeederSetPercentOutputCommand(subsystems.feeder));
        oi.bindFeederDisable(new FeederStopCommand(subsystems.feeder));

        oi.bindVisionFullEnable(new ScheduleCommand(
            new LimelightSetTrackingModeCommand(subsystems.vision.turretLimelight),
            new FlywheelManagerAutoTargetCommand(subsystems.managers.flywheelVelocityManager, subsystems.vision.turretLimelight::getDistanceEstimate),
            new HoodManagerAutoTargetCommand(subsystems.managers.hoodPositionManager, subsystems.vision.turretLimelight::getDistanceEstimate),
            new TurretManagerAutoTargetCommand(subsystems.managers.turretPositionManager, subsystems.vision.turretLimelight::getDistanceEstimate)
        ));

        oi.bindVisionFullDisable(new ScheduleCommand(
            new LimelightSetDriverModeCommand(subsystems.vision.turretLimelight),
            new FlywheelManagerSetTargetVelocityCommand(subsystems.managers.flywheelVelocityManager, subsystems.managers.flywheelVelocityManager.getTargetRPM()),
            new HoodManagerSetTargetPositionCommand(subsystems.managers.hoodPositionManager, subsystems.managers.hoodPositionManager.getTargetDegrees()),
            new TurretManagerSetTargetPositionCommand(subsystems.managers.turretPositionManager, subsystems.managers.turretPositionManager.getTargetDegrees())
        ));

        oi.bindVisionTurretEnable(new ScheduleCommand(
            new LimelightSetTrackingModeCommand(subsystems.vision.turretLimelight),
            new TurretManagerAutoTargetCommand(subsystems.managers.turretPositionManager, subsystems.vision.turretLimelight::getDistanceEstimate)
        ));

        oi.bindCancelAll(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    }
    */
}
