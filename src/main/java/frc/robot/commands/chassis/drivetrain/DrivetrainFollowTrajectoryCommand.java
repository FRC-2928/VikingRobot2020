package frc.robot.commands.chassis.drivetrain;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.chassis.DrivetrainSubsystem;

public class DrivetrainFollowTrajectoryCommand extends RamseteCommand {

  public DrivetrainFollowTrajectoryCommand(DrivetrainSubsystem drivetrain, Trajectory trajectory) {
    super(trajectory, drivetrain::getPose, new RamseteController(), drivetrain.getKinematics(), drivetrain::setLeftRightVelocity, drivetrain);
  }
  
}
