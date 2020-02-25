package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.control.SetPositionCommand;
import frc.robot.commands.control.SetVelocityCommand;
import frc.robot.commands.intake.FastForwardFeeder;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.types.DistanceMap;
import frc.robot.utilities.Limelight;


// Shoot balls
public class ShootBallsCommand extends SequentialCommandGroup {

    // private double m_hoodDegrees;
    // private double m_flywheelRPM;
    // private HoodSubsystem m_hood;
    // private FlywheelSubsystem m_flywheel;
    // private FeederSubsystem m_feeder;

    public ShootBallsCommand(HoodSubsystem hood, 
                            Limelight limelight, // doubleSupplier
                            FlywheelSubsystem flywheel, 
                            FeederSubsystem feeder) {
        addRequirements(hood, flywheel, feeder);   
        // m_hood = hood;
        // m_flywheel = flywheel;
        // m_feeder = feeder;

        double distance = limelight.getTargetDistance();
        // Add reference calculation here....
        double hoodDegrees = calculateHoodDegrees(distance);
        double flywheelRPM = calculateFlywheelRPM(distance);

        addCommands(
            // Set the hood
            new SetPositionCommand(hood, hoodDegrees),
    
            // Spin up the flywheel
            new SetVelocityCommand(flywheel, flywheelRPM),
    
            // Drive backward the specified distance
            new FastForwardFeeder(feeder));
      }

    private static double calculateHoodDegrees(double distance) {
        double degrees = DistanceMap.getInstance().getHoodDegrees(distance);
        return degrees;
    }

    private static double calculateFlywheelRPM(double distance) {
        double rpm = DistanceMap.getInstance().getFlywheelRPM(distance);
        return rpm;
    }
}    