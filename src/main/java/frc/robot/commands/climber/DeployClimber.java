package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberState;

public class DeployClimber extends CommandBase {
    // The subsystem the command runs on
    private final ClimberSubsystem m_climber;

    public DeployClimber(ClimberSubsystem subsystem) {
        m_climber = subsystem;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_climber.deployToTop();
    }

    @Override
    public boolean isFinished() {
        return m_climber.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {

        if (interrupted) {
            m_climber.setClimberState(ClimberState.INTERRUPTED);
        } else {
            // Ready to latch on. May want to do this in a separate command?  
            m_climber.setClimberState(ClimberState.READY_TO_LATCH);
        }       
    }
}