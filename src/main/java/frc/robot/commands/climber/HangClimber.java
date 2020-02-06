package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem.BrakeState;

/**
 * Stops the feeder subsystem
 */
public class HangClimber extends CommandBase {
    // The subsystem the command runs on
    private final ClimberSubsystem m_climber;
    private double m_power;

    public HangClimber(ClimberSubsystem subsystem) {
        m_climber = subsystem;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.setBrakePosition(BrakeState.OFF);
        m_climber.setClimberState(ClimberState.HANGING);

        m_power = PIDConstants.kCLimberPower;
    }

    @Override
    public void execute() {
        m_climber.setElevatorPower(m_power);
    }

    @Override
    public void end(boolean interrupted) {

        m_climber.setBrakePosition(BrakeState.ON);

        if (interrupted) {
            m_climber.setClimberState(ClimberState.INTERRUPTED);
        } else {
            // Ready to latch on. May want to do this in a separate command?  
            m_climber.setClimberState(ClimberState.HUNG);
        }       
    }
}