package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.FastForwardFeeder;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.utilities.Limelight;


// Shoot balls
public class ShootBallsCommand extends SequentialCommandGroup {

    public ShootBallsCommand(HoodSubsystem hood, 
                            Limelight limelight,
                            FlywheelSubsystem flywheel, 
                            FeederSubsystem feeder) {
                                
        double distance = limelight.getTargetDistance();
        // Add reference calculation here....
        double hoodDegrees = 0;
        double flywheelRPM = 0;

        addCommands(
            // Set the hood
            new SetHoodDegrees(hood, hoodDegrees),
    
            // Spin up the flywheel
            new SpinUpFlywheel(flywheel, flywheelRPM),
    
            // Drive backward the specified distance
            new FastForwardFeeder(feeder));
      }
}    