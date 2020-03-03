package frc.robot.commands.chassis.drivetrain;

import java.util.function.Supplier;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.ArcadeDriveValue;

import frc.robot.subsystems.chassis.DrivetrainSubsystem;

public class DrivetrainArcadeDriveCommand extends SuppliedCommand<ArcadeDriveValue> {
  private DrivetrainSubsystem m_drivetrain;

  public DrivetrainArcadeDriveCommand(DrivetrainSubsystem drivetrain) {
    this(drivetrain, () -> new ArcadeDriveValue(0.0, 0.0));
  }

  public DrivetrainArcadeDriveCommand(DrivetrainSubsystem drivetrain, Supplier<ArcadeDriveValue> supplier) {
    super(supplier);
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }


  @Override
  public void execute() {
    var arcadeDriveValue = m_supplier.get();
    m_drivetrain.arcadeDrive(arcadeDriveValue.move, arcadeDriveValue.rotate);
  }
}
