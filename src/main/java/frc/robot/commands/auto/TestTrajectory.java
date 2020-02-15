package frc.robot.commands.auto;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * An example command that uses an example subsystem.
 */
public class TestTrajectory extends CommandBase {
    
    private final DrivetrainSubsystem m_robotDrive;
    private Trajectory m_trajectory;

    
    public TestTrajectory(DrivetrainSubsystem subsystem) {
    m_robotDrive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DrivetrainConstants.ksVolts,
                                   DrivetrainConstants.kvVoltSecondsPerMeter,
                                   DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
        DrivetrainConstants.kDriveKinematics,
        10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivetrainConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    String trajectoryJSON = "trajectories/test.wpilib.json";
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RamseteCommand ramseteCommand = new RamseteCommand(
        // Where we're going
        m_trajectory,

        // Where we are currently.  Input from the drivetrain
        m_robotDrive::getPose,

        // The controller. Computes the wheel speeds for the next spline
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
        
        // Kinematics, defined in the Constants file
        DrivetrainConstants.kDriveKinematics,

        // Consumed in the drivetrain to pass to PID loop
        m_robotDrive::outputMetersPerSecond,

        // Required subsystem
        m_robotDrive
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}