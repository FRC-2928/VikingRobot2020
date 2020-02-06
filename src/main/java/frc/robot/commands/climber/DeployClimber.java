package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem.BrakeState;

public class DeployClimber extends CommandBase {
    // The subsystem the command runs on
    private final ClimberSubsystem m_climber;
    private double m_positionSetpoint;

    public DeployClimber(ClimberSubsystem subsystem) {
        m_climber = subsystem;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.setBrakePosition(BrakeState.OFF);
        m_climber.setClimberState(ClimberState.DEPLOYING);

        m_positionSetpoint = m_climber.calculateSetpoint();
    }

    @Override
    public void execute() {
        m_climber.setElevatorPosition(m_positionSetpoint);
    }

    @Override
    public void end(boolean interrupted) {

        m_climber.setBrakePosition(BrakeState.ON);

        if (interrupted) {
            m_climber.setClimberState(ClimberState.INTERRUPTED);
        } else {
            // Ready to latch on. May want to do this in a separate command?  
            m_climber.setClimberState(ClimberState.READY_TO_LATCH);
        }       
    }
}