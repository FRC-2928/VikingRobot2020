package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.intake.StartFeeder;
import frc.robot.commands.shooter.ShooterManagerSetReference;
import frc.robot.commands.turret.TrackTargetCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterManager;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.types.DistanceMap;
import frc.robot.utilities.Limelight;

/**
 * NOTE: REALLY DUMB AUTO!!!! 
 * ONLY TO BE USED IF OUR ACTUAL AUTOS DON'T WORK!!!!
 */

public class ShootThreeThenTrench extends ParallelCommandGroup {
  public ShootThreeThenTrench(DrivetrainSubsystem drivetrain,
  IntakeSubsystem intake, FlywheelSubsystem flywheel, HoodSubsystem hood, 
  TurretSubsystem turret, FeederSubsystem feeder, Limelight limelight, 
  ShooterManager shootermanager, DistanceMap distanceMap) {
    super(
      new ParallelCommandGroup(
        new TrackTargetCommand(turret, drivetrain, limelight),

        new ShooterManagerSetReference(shootermanager, 
        distanceMap.getHoodDegrees(limelight.getTargetDistance()),
        distanceMap.getFlywheelRPM(limelight.getTargetDistance())),

        new RunCommand(
          () -> {
              flywheel.setVelocity(shootermanager.getFlywheelReference());
              hood.setHoodDegrees(shootermanager.getHoodReference());
          }, 
        flywheel,hood),

        new SequentialCommandGroup(
          new WaitCommand(2),
          new RunCommand(feeder::startFeeder, feeder).withTimeout(2),
          new ParallelCommandGroup(
            new RunCommand(intake::pickupFromGround, intake),
            new StartFeeder(feeder),
            new Drive(drivetrain, 0.7, 0)
          ).withTimeout(3),
          new WaitCommand(0.5),
          new Drive(drivetrain, -0.65, 0).withTimeout(2.5),
          new WaitCommand(1),
          new RunCommand(feeder::startFeeder, feeder)
        )
      )
    );
  }
}
