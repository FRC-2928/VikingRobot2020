package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class DrivetrainFollowTrajectoryCommand extends RamseteCommand {
  /**
   * Creates a new DrivetrainFollowTrajectoryCommand.
   */
  public DrivetrainFollowTrajectoryCommand(DrivetrainSubsystem drivetrain, Trajectory trajectory) {
    super(
      trajectory,
      drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      DrivetrainConstants.kDriveKinematics,
      drivetrain::outputMetersPerSecond,
      drivetrain
    );
  }

}
