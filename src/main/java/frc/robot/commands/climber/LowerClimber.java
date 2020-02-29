package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberState;

/**
 * Stops the feeder subsystem
 */
public class LowerClimber extends CommandBase {
    // The subsystem the command runs on
    private final ClimberSubsystem m_climber;
    private double m_power;

    public LowerClimber(ClimberSubsystem subsystem) {
        m_climber = subsystem;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.setClimberState(ClimberState.LOWERING);
        m_power = ClimberConstants.kClimberPower;
    }

    @Override
    public void execute() {
        m_climber.setSolenoid(true);
        m_climber.setPower(m_power);
    }

    @Override
    public void end(boolean interrupted) {

        if (interrupted) {
            m_climber.setClimberState(ClimberState.INTERRUPTED);
        } else {
            // Ready to latch on. May want to do this in a separate command?  
            m_climber.setClimberState(ClimberState.ASSENT_COMPLETE);
        }       
    }
}