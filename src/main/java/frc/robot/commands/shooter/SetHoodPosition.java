package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.types.DistanceMap;
import frc.robot.utilities.Limelight;

/*
 * Spins up the hood to desired RPM
 */

public class SetHoodPosition extends CommandBase {

    // The subsystem the command runs on
    private final HoodSubsystem m_hood;
    private double m_hoodDegrees;

    public SetHoodPosition(HoodSubsystem hood, Limelight limelight) {
        addRequirements(hood);
        m_hood = hood;
        double distance = limelight.getTargetDistance();
        m_hoodDegrees = calculatehoodDegrees(distance);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_hood.setPosition(m_hoodDegrees);
    }

    // Calculate the Hood degrees based on the distance
    private static double calculatehoodDegrees(double distance) {
        double degrees = DistanceMap.getInstance().getHoodDegrees(distance);
        return degrees;
    }

}