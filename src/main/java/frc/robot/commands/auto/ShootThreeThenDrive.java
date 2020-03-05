package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.intake.FastForwardFeeder;
import frc.robot.commands.shooter.SetHoodPosition;
import frc.robot.commands.shooter.SetSetpointShooting;
import frc.robot.commands.shooter.SetShooterVision;
import frc.robot.commands.shooter.ShooterAtReference;
import frc.robot.commands.shooter.SpinUpFlywheel;
import frc.robot.commands.shooter.SetSetpointShooting.ShooterSetpoint;
import frc.robot.commands.turret.TurretAtReference;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.types.LimelightData;
import frc.robot.utilities.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootThreeThenDrive extends SequentialCommandGroup {
  public ShootThreeThenDrive(DrivetrainSubsystem drivetrain,
  FlywheelSubsystem flywheel, HoodSubsystem hood, TurretSubsystem turret,
  FeederSubsystem feeder, Limelight limelight) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ParallelCommandGroup(
        new SetSetpointShooting(flywheel, hood, ShooterSetpoint.INITIATION_LINE),
        new SequentialCommandGroup(
          new WaitCommand(3),
          new RunCommand(feeder::startFeeder, feeder).withTimeout(5)
        )
      ).withTimeout(8),
      new Drive(drivetrain, 0.4, 0).withTimeout(3)
    );
  }
}