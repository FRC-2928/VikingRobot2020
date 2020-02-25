package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.types.DistanceMap;
import frc.robot.utilities.Limelight;

/*
 * Spins up the flywheel to desired RPM
 */

public class SpinUpFlywheel extends CommandBase {

    // The subsystem the command runs on
    private final FlywheelSubsystem m_flywheel;
    private double m_flywheelRPM;

    public SpinUpFlywheel(FlywheelSubsystem flywheel, Limelight limelight) {
        addRequirements(flywheel);
        m_flywheel = flywheel;
        double distance = limelight.getTargetDistance();
        m_flywheelRPM = calculateFlywheelRPM(distance);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_flywheel.setFlywheelRPM(m_flywheelRPM);
    }

    // Calculate the RPM based on the distance
    private static double calculateFlywheelRPM(double distance) {
        double rpm = DistanceMap.getInstance().getFlywheelRPM(distance);
        return rpm;
    }

}