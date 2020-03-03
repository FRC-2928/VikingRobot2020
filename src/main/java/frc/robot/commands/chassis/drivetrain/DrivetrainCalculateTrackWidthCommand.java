package frc.robot.commands.chassis.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.DrivetrainSubsystem;

/*
 * Measure the effective track width of the robot drivetrain.
 * This is done by making the following measurements:
 *    1) The rotation of the robot via gyro, in radians, called theta.
 *    2) The arcLength of the wheel path via encoders, in meters, called s.
 * Then, we solve the the radius r of the circle using the equation:
 *    r = s / theta
 * The track width is two times this radius.
 */
public class DrivetrainCalculateTrackWidthCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrain;
  private DoubleSupplier m_rotateSupplier;

  public DrivetrainCalculateTrackWidthCommand(DrivetrainSubsystem drivetrain, DoubleSupplier rotateSupplier) {
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_rotateSupplier = rotateSupplier;
  }

  @Override
  public void initialize() {
    m_drivetrain.resetGyro();
    m_drivetrain.resetEncoders();
  }

  @Override
  public void execute() {
    // Rotate the robot with the double supplier.
    m_drivetrain.arcadeDrive(0.0, m_rotateSupplier.getAsDouble());

    // Average the encoder position to get the length of the arc.
    double leftPos = m_drivetrain.getLeftPosition();
    double rightPos = m_drivetrain.getRightPosition();
    double arcLength = (Math.abs(leftPos) + Math.abs(rightPos)) / 2.0;

    // Get angle in radians from gyro.
    double angle = m_drivetrain.getHeading();

    // Compute track width.
    double trackWidth = 0.0;
    if (angle != 0.0) {
      double radius = Math.abs(arcLength / angle);
      trackWidth = 2.0 * radius;
    }
    SmartDashboard.putNumber("TrackWidth", trackWidth);
  }
}
