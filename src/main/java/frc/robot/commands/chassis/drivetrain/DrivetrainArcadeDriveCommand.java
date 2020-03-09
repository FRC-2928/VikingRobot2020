package frc.robot.commands.chassis.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.DrivetrainSubsystem;

public class DrivetrainArcadeDriveCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrain;
  private DoubleSupplier m_moveSupplier;
  private DoubleSupplier m_rotateSupplier;

  public DrivetrainArcadeDriveCommand(DrivetrainSubsystem drivetrain, DoubleSupplier moveSupplier, DoubleSupplier rotateSupplier) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_moveSupplier = moveSupplier;
    m_rotateSupplier = rotateSupplier;
  }

  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_moveSupplier.getAsDouble(), m_rotateSupplier.getAsDouble());
  }
}
