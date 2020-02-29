package frc.robot.commands.drive;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.ArcadeDriveValue;

import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class DrivetrainArcadeDriveCommand extends SuppliedCommand<ArcadeDriveValue> {
  private DrivetrainSubsystem m_drivetrain;

  public DrivetrainArcadeDriveCommand(DrivetrainSubsystem drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  @Override
  public void execute() {
    var arcadeDriveValue = m_supplier.get();
    m_drivetrain.arcadeDrive(arcadeDriveValue.move, arcadeDriveValue.rotate);
  }
}
