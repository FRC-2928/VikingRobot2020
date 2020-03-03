package frc.robot.commands.chassis.transmission;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.chassis.TransmissionSubsystem;

public class TransmissionSetLowGearCommand extends InstantCommand {
  public TransmissionSetLowGearCommand(TransmissionSubsystem transmission) {
    super(() -> transmission.setLowGear(), transmission);
  }
}
