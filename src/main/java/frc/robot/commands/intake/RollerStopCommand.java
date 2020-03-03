package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intake.RollerSubsystem;

public class RollerStopCommand extends RunCommand {
  public RollerStopCommand(RollerSubsystem roller) {
    super(() -> roller.stop(), roller);
  }
}
