package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.ArmSubsystem;
import frc.robot.subsystems.intake.RollerSubsystem;

public class OpenIntakeCommand extends SequentialCommandGroup {
  public OpenIntakeCommand(RollerSubsystem roller, ArmSubsystem arm) {
    super(
      new InstantCommand(arm::openIntake, arm),
      new RunCommand(roller::reverseMotor, roller)
    );
  }
}
