package frc.robot.commands.control;

import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ballardrobotics.subsystems.SmartSubsystem;

/*
 * Move the subsystem to the requested position
 */
public class SetVelocityCommand extends CommandBase {
    private SmartSubsystem m_subsystem;
    private Twist2d m_twist;
    private Boolean m_stopIfFinished;

    public SetVelocityCommand(SmartSubsystem subsystem, Twist2d velocity) {
        this(subsystem, velocity, false);
    }

    public SetVelocityCommand(SmartSubsystem subsystem, Twist2d velocity, Boolean stopIfFinished) {
        addRequirements(subsystem);
        m_subsystem = subsystem;
        m_twist = velocity;
        m_stopIfFinished = stopIfFinished;
    }

    @Override
    public void execute() {
        m_subsystem.setVelocity(m_twist);
    }

    public boolean isFinished() {
        if (m_stopIfFinished) {
            return m_subsystem.atReference();
        } 
        return false;
    }
}
