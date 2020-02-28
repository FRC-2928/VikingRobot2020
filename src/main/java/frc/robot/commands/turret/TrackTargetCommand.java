package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.turret.TargetEstimator;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.types.LimelightData;
import frc.robot.types.TargetEstimate;
import frc.robot.utilities.Limelight;

// Turret track vision target
public class TrackTargetCommand extends CommandBase {

    private TurretSubsystem m_turret;
    private DrivetrainSubsystem m_drivetrain;
    private Limelight m_limelight;
    private LimelightData m_limelightData;
    private TargetEstimate m_targetEstimate;
    private TargetEstimator m_targetEstimator = new TargetEstimator();
    

    public TrackTargetCommand(TurretSubsystem turret, 
                              DrivetrainSubsystem drivetrain,
                              Limelight limelight) {
        addRequirements(turret);

        m_turret = turret;
        m_drivetrain = drivetrain;
        m_limelight = limelight;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_limelight.setPipeline(1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        m_limelightData = m_turret.getLimelightData();
        m_targetEstimator.update(
            m_drivetrain.getPose(),
            m_turret.getTurretFieldDegrees(),
            m_limelightData
        );
        m_targetEstimate = m_targetEstimator.getEstimate();

        boolean isTargetFound = m_limelightData.getTargetFound();
        double visionReference = m_turret.getTurretDegrees() - m_limelightData.getHorizontalOffset();
        
        if(isTargetFound){
            // Keep it on target
            m_turret.setPosition(visionReference);
        }
        else {
            if(m_targetEstimate.isValid()){
                // Try using the last estimate of where the target was
                double reference = m_targetEstimate.getAngle();
                m_turret.setPosition(reference);
            }
            else{
                // No estimate so search for the target
                m_turret.searchForTarget();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}