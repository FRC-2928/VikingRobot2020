package frc.robot.commands.drive;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.TankDriveValue;

import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class DrivetrainTankDriveCommand extends SuppliedCommand<TankDriveValue> {
  private DrivetrainSubsystem m_drivetrain;

  public DrivetrainTankDriveCommand(DrivetrainSubsystem drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  @Override
  public void execute() {
    var tankDriveValue = m_supplier.get();
    m_drivetrain.tankDrive(tankDriveValue.left, tankDriveValue.right);
  }
}
