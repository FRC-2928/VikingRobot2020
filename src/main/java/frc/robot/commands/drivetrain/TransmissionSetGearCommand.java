package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.TransmissionSubsystem;
import frc.robot.subsystems.drivetrain.TransmissionSubsystem.GearState;

public class TransmissionSetGearCommand extends InstantCommand {
  public TransmissionSetGearCommand(TransmissionSubsystem transmission, GearState desiredState) {
    super(() -> transmission.setGearState(desiredState), transmission);
  }
}
