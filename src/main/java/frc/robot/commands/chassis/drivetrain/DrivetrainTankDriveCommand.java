package frc.robot.commands.chassis.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.DrivetrainSubsystem;

public class DrivetrainTankDriveCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrain;
  private DoubleSupplier m_leftSupplier;
  private DoubleSupplier m_rightSupplier;

  public DrivetrainTankDriveCommand(DrivetrainSubsystem drivetrain, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_leftSupplier = leftSupplier;
    m_rightSupplier = rightSupplier;
  }

  @Override
  public void execute() {
    m_drivetrain.tankDrive(m_leftSupplier.getAsDouble(), m_rightSupplier.getAsDouble());
  }
}
