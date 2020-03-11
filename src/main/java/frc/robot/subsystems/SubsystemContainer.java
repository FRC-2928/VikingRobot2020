package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.TransmissionSubsystem;
import frc.robot.subsystems.intake.ArmSubsystem;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterManager;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.types.DistanceMap;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.Limelight.Limelights;
import frc.robot.utilities.Pigeon;

/**
 * Contains all subsystem classes
 */

public class SubsystemContainer {
    public final TransmissionSubsystem transmission = new TransmissionSubsystem();
    public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(transmission::getGearState);
    public final FlywheelSubsystem flywheel = new FlywheelSubsystem();
    public final HoodSubsystem hood = new HoodSubsystem();
    public final RollerSubsystem roller = new RollerSubsystem();
    public final ArmSubsystem arm = new ArmSubsystem();
    // public final ControlPanelSubsystem controlPanel = new ControlPanelSubsystem();
    // public final ClimberSubsystem climber = new ClimberSubsystem();
    public final FeederSubsystem feeder = new FeederSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();
    public final Pigeon pigeon = new Pigeon();
    public final ShooterManager shooterManager = new ShooterManager();
    public final Limelight turretLimelight = new Limelight(Limelights.TURRET);
    public final DistanceMap distanceMap = DistanceMap.getInstance();

    public SubsystemContainer(){
        DistanceMap.getInstance().loadMaps();

        //Default commands
        drivetrain.setDefaultCommand(new RunCommand(drivetrain::stop, drivetrain));
        flywheel.setDefaultCommand(new RunCommand(flywheel::stop, flywheel));
        hood.setDefaultCommand(new RunCommand(hood::stop, hood));
        roller.setDefaultCommand(new RunCommand(roller::stop, roller));
        arm.setDefaultCommand(new RunCommand(arm::stowIntake, arm));
        feeder.setDefaultCommand(new RunCommand(feeder::stop, feeder));
        turret.setDefaultCommand(new RunCommand(turret::stop, turret));
    }
}