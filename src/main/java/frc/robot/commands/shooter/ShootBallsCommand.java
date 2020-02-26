package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.control.SetPositionCommand;
import frc.robot.commands.control.SetVelocityCommand;
import frc.robot.commands.intake.FastForwardFeeder;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.types.DistanceMap;
import frc.robot.utilities.Limelight;


// Shoot balls
// public class ShootBallsCommand extends ParallelDeadlineGroup {

    // private double m_hoodDegrees;
    // private double m_flywheelRPM;
    // private HoodSubsystem m_hood;
    // private FlywheelSubsystem m_flywheel;
    // private FeederSubsystem m_feeder;

    // public ShootBallsCommand(FastForwardFeeder fastForwardFeeder, 
    //                          SetHoodPosition setHoodPosition, 
    //                          SpinUpFlywheel setFlywheel) {

    //     addCommands(
    //         // Set the hood
    //         new fastForwardFeeder(feeder),
    
    //         // Spin up the flywheel
    //         new SetVelocityCommand(flywheel, flywheelRPM),
    
    //         // Drive backward the specified distance
    //         new FastForwardFeeder(feeder));
    //   }

    
// }    