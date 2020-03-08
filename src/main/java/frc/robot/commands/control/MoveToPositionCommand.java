package frc.robot.commands.control;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import java.util.function.Supplier;
import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.subsystems.SmartSubsystem;
import org.ballardrobotics.types.supplied.Pose2dValue;

/*
 * Move the subsystem to the requested position
 */
public class MoveToPositionCommand extends SuppliedCommand<Pose2dValue> {
    private SmartSubsystem m_subsystem;
    private Boolean m_stopIfFinished;

    public MoveToPositionCommand(SmartSubsystem subsystem, Pose2d pose) {
        this(subsystem, () -> new Pose2dValue(pose), false);
    }

    public MoveToPositionCommand(SmartSubsystem subsystem, Supplier<Pose2dValue> supplier, Boolean stopIfFinished) {
        super(supplier);
        addRequirements(subsystem);

        m_subsystem = subsystem;
        m_stopIfFinished = stopIfFinished;
    }

    @Override
    public void initialize() {
        m_subsystem.setPosition(m_supplier.get().value);
    }

    @Override
    public void execute() {
        m_subsystem.moveToPosition();
    }

    public boolean isFinished() {
        if (m_stopIfFinished) {
            return m_subsystem.atReference();
        } 
        return false;
    }
}
