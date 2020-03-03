package frc.robot;

import org.ballardrobotics.sensors.fakes.FakeIMU;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.subsystems.Subsystems;

public class RobotContainer {
  private final Subsystems m_subsystems;
  private final Compressor m_compressor;
  private boolean m_initialized = false;

  public RobotContainer() {
    m_subsystems = new Subsystems();
    m_compressor = new Compressor();

    var startCompressor = new InstantCommand(() -> m_compressor.start());
    var stopCompressor = new InstantCommand(() -> m_compressor.stop());
    var layout = Shuffleboard.getTab("Pneumatics").getLayout("Compressor", BuiltInLayouts.kList).withSize(2, 2).withPosition(0, 0);
    layout.add("start", startCompressor);
    layout.add("stop", stopCompressor);
    layout.addBoolean("enabled", m_compressor::enabled);
    startCompressor.schedule();

    var turretFieldRelativeGyro = new FakeIMU();
    Shuffleboard.getTab(ShuffleboardConstants.kShooterTab).add("turret_field_relative", turretFieldRelativeGyro);
    new RunCommand(
      () -> {
        double turretAngle = m_subsystems.shooter.turret.getMeasuredPosition();
        double chassisAngle = m_subsystems.chassis.drivetrain.getHeading();
        double shiftedChassisAngle = chassisAngle - 180.0;
        double fieldRelativeAngle = (turretAngle - shiftedChassisAngle) % 360;
        if (fieldRelativeAngle < 0) {
          fieldRelativeAngle += 360;
        }
        turretFieldRelativeGyro.setAngle(fieldRelativeAngle);
      }
    ).schedule();
  }

  public void onInit() {
    if (m_initialized) {
      return;
    }
    m_initialized = true;

    m_subsystems.chassis.transmission.setLowGear();
    m_subsystems.chassis.drivetrain.setHeading(180.0);
  }

  public void configureButtonBindings() {

  }

  public Command getAutonomousCommand() {
    return new PrintCommand("auto not implemented");
  }
}
