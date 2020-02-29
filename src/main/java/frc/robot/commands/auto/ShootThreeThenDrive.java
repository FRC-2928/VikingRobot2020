package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.shooter.SetHoodPosition;
import frc.robot.commands.shooter.SetShooterVision;
import frc.robot.commands.shooter.SpinUpFlywheel;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
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
  Limelight limelight) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ParallelCommandGroup(
        new SpinUpFlywheel(flywheel, limelight),
        new SetHoodPosition(hood, limelight)
      ).withTimeout(7),
      new Drive(drivetrain, 0.5, 0).withTimeout(3)
    );
  }
}
