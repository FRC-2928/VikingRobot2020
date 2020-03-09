package frc.robot.commands.control;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ballardrobotics.subsystems.SmartSubsystem;

/*
 * Move the subsystem to the requested position
 */
public class SetPositionCommand extends CommandBase {
    private SmartSubsystem m_subsystem;
    private Pose2d m_pose;
    private Boolean m_stopIfFinished;

    public SetPositionCommand(SmartSubsystem subsystem, Pose2d pose) {
        this(subsystem, pose, false);
    }

    public SetPositionCommand(SmartSubsystem subsystem, Pose2d pose, Boolean stopIfFinished) {
        addRequirements(subsystem);
        m_subsystem = subsystem;
        m_pose = pose;
        m_stopIfFinished = stopIfFinished;
    }

    @Override
    public void initialize() {
        m_subsystem.setPositionReference(m_pose);
    }

    @Override
    public void execute() {
        m_subsystem.setPosition();
    }

    public boolean isFinished() {
        if (m_stopIfFinished) {
            return m_subsystem.atReference();
        } 
        return false;
    }
}
