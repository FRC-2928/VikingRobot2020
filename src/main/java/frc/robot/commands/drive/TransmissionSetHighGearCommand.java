package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.TransmissionSubsystem;

public class TransmissionSetHighGearCommand extends CommandBase {
  private TransmissionSubsystem m_transmission;

  public TransmissionSetHighGearCommand(TransmissionSubsystem transmission) {
    addRequirements(transmission);

    m_transmission = transmission;
  }

  @Override
  public void initialize() {
    m_transmission.setHighGear();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
