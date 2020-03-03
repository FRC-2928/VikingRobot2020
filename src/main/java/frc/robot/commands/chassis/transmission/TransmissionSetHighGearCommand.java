package frc.robot.commands.chassis.transmission;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.chassis.TransmissionSubsystem;

public class TransmissionSetHighGearCommand extends InstantCommand {
  public TransmissionSetHighGearCommand(TransmissionSubsystem transmission) {
    super(() -> transmission.setHighGear(), transmission);
  }
}
