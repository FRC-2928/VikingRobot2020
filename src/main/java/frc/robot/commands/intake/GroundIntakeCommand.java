package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.ArmSubsystem;
import frc.robot.subsystems.intake.RollerSubsystem;

public class GroundIntakeCommand extends SequentialCommandGroup {
  public GroundIntakeCommand(RollerSubsystem roller, ArmSubsystem arm) {
    super(
      new InstantCommand(arm::groundPickup, arm),
      new RunCommand(roller::startMotor, roller)
    );
  }
}
