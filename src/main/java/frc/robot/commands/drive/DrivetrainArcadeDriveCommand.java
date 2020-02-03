package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class DrivetrainArcadeDriveCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrain;
  private DoubleSupplier m_moveSupplier, m_rotateSupplier;

  public DrivetrainArcadeDriveCommand(DrivetrainSubsystem drivetrain, DoubleSupplier moveSupplier, DoubleSupplier rotateSupplier) {
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_moveSupplier = moveSupplier;
    m_rotateSupplier = rotateSupplier;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double move = m_moveSupplier.getAsDouble();
    double rotate = m_rotateSupplier.getAsDouble();
    m_drivetrain.arcadeDrive(move, rotate);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
