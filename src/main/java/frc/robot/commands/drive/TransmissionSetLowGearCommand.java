package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.TransmissionSubsystem;

public class TransmissionSetLowGearCommand extends CommandBase {
  private TransmissionSubsystem m_transmission;

  public TransmissionSetLowGearCommand(TransmissionSubsystem transmission) {
    addRequirements(transmission);

    m_transmission = transmission;
  }

  @Override
  public void initialize() {
    m_transmission.setLowGear();
  }
}
