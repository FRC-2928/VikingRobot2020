package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.ArmSubsytem;
import frc.robot.subsystems.intake.ArmSubsytem.State;

public class ArmSetStateCommand extends InstantCommand {
  public ArmSetStateCommand(ArmSubsytem arm, State state) {
    super(() -> arm.setState(state), arm);
  }
}
