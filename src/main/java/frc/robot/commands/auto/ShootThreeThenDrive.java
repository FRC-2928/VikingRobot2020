package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.shooter.SetSetpointShooting;
import frc.robot.commands.shooter.SetSetpointShooting.ShooterSetpoint;
import frc.robot.subsystems.SubsystemContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootThreeThenDrive extends SequentialCommandGroup {
  public ShootThreeThenDrive(SubsystemContainer subsystems) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ParallelCommandGroup(
        new SetSetpointShooting(subsystems.flywheel, subsystems.hood, ShooterSetpoint.INITIATION_LINE),
        new SequentialCommandGroup(
          new WaitCommand(3),
          new RunCommand(subsystems.feeder::startFeeder, subsystems.feeder).withTimeout(5)
        )
      ).withTimeout(8),
      new Drive(subsystems.drivetrain, 0.4, 0).withTimeout(3)
    );
  }
}