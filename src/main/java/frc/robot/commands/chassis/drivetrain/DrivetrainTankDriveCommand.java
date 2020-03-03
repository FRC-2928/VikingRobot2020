package frc.robot.commands.chassis.drivetrain;

import java.util.function.Supplier;

import org.ballardrobotics.commands.SuppliedCommand;
import org.ballardrobotics.types.supplied.TankDriveValue;

import frc.robot.subsystems.chassis.DrivetrainSubsystem;

public class DrivetrainTankDriveCommand extends SuppliedCommand<TankDriveValue> {
  private DrivetrainSubsystem m_drivetrain;

  public DrivetrainTankDriveCommand(DrivetrainSubsystem drivetrain) {
    this(drivetrain, () -> new TankDriveValue(0.0, 0.0));
  }

  public DrivetrainTankDriveCommand(DrivetrainSubsystem drivetrain, Supplier<TankDriveValue> supplier) {
    super(supplier);
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  @Override
  public void execute() {
    var tankDriveValue = m_supplier.get();
    m_drivetrain.tankDrive(tankDriveValue.left, tankDriveValue.right);
  }
}
