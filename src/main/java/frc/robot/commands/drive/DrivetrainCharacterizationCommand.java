package frc.robot.commands.drive;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class DrivetrainCharacterizationCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrain;
  private NetworkTableEntry m_autoSpeedEntry;
  private NetworkTableEntry m_telemetryEntry;
  private NetworkTableEntry m_rotateEntry;
  private Number[] m_numberArray;

  public DrivetrainCharacterizationCommand(DrivetrainSubsystem drivetrain) {
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    m_telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    m_rotateEntry = NetworkTableInstance.getDefault().getEntry("robot/rotate");
    m_numberArray = new Number[10];
  }

  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
    m_drivetrain.resetEncoders();
    m_drivetrain.resetGyro();
  }

  @Override
  public void execute() {
    double autospeed = m_autoSpeedEntry.getDouble(0.0);
    double autoVoltage = autospeed * 12.0;

    double leftVoltage = m_rotateEntry.getBoolean(false) ? -1 : 1 * autoVoltage;
    double rightVoltage = autoVoltage;

    m_drivetrain.setLeftRightVoltage(leftVoltage, rightVoltage);

    m_numberArray[0] = Timer.getFPGATimestamp();
    m_numberArray[1] = RobotController.getBatteryVoltage();
    m_numberArray[2] = autospeed;
    m_numberArray[3] = m_drivetrain.getLeftVoltage();
    m_numberArray[4] = m_drivetrain.getRightVoltage();
    m_numberArray[5] = m_drivetrain.getLeftPosition();
    m_numberArray[6] = m_drivetrain.getRightPosition();
    m_numberArray[7] = m_drivetrain.getLeftVelocity();
    m_numberArray[8] = m_drivetrain.getRightVelocity();
    m_numberArray[9] = Math.toRadians(m_drivetrain.getHeading());
    m_telemetryEntry.setNumberArray(m_numberArray);
  }

  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().setUpdateRate(0.10);
    m_drivetrain.stop();
  }
}
