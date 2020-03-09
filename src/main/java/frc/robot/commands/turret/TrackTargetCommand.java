package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.turret.TargetEstimator;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.TurretSafetyRangeState;
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
    private double m_correctingAngle;
    private State m_currentState;
    private static boolean wasCorrecting = false;

    private enum State{
        UNKNOWN, FIELD_RELATIVE, TRACKING, CORRECTING;
    }

    public TrackTargetCommand(TurretSubsystem turret, 
                              DrivetrainSubsystem drivetrain,
                              Limelight limelight) {
        addRequirements(turret);

        SmartDashboard.putNumber("Skew factor", 0);

        m_turret = turret;
        m_drivetrain = drivetrain;
        m_limelight = limelight;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_limelight.setPipeline(2);
        m_currentState = State.UNKNOWN;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putString("Turret Command State", m_currentState.toString());
        SmartDashboard.putNumber("Turret Command Correcting Angle", m_correctingAngle);
        SmartDashboard.putBoolean("Turret Command Was Correcting", wasCorrecting);

        m_limelightData = m_turret.getLimelightData();
        m_targetEstimator.update(
            m_drivetrain.getPose(),
            m_turret.getTurretFieldDegrees(),
            m_limelightData
        );
        m_targetEstimate = m_targetEstimator.getEstimate();

        boolean isTargetFound = m_limelightData.getTargetFound();
        
        if(m_currentState == State.CORRECTING){
            double error = Math.abs(m_turret.getPosition() - m_correctingAngle);
            wasCorrecting = true;
            m_turret.setPosition(m_correctingAngle);
            if(error < 180 && isTargetFound){
                m_currentState = State.TRACKING;
            }
            else if(error > 10){
                return; //Continue correcting until within 10 degrees
            }
        }

        // visionReference += m_limelightData.getSkew() * SmartDashboard.getNumber("Skew factor", 0);

        if(isTargetFound){
            double visionReference = m_turret.getTurretDegrees() - m_limelightData.getHorizontalOffset();
    
            if(!m_turret.isAngleValid(visionReference)){
                m_correctingAngle = m_turret.getValidAngle(visionReference);
                m_currentState = State.CORRECTING;
                return;
            }
            
            // Keep it on target
            m_turret.setPosition(visionReference);
            m_currentState = State.TRACKING;
        }
        // else {
        //     if(m_targetEstimate.isValid()){
        //         // Try using the last estimate of where the target was
        //         double reference = m_targetEstimate.getAngle();
        //         m_turret.setPosition(reference);
        //     }
        else{
            // No estimate so search for the target
            m_turret.searchForTarget();
            m_currentState = State.FIELD_RELATIVE;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
        m_limelight.setPipeline(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}