/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.commands.drive.DrivetrainArcadeDriveCommand;
import frc.robot.commands.drive.TransmissionSetHighGearCommand;
import frc.robot.commands.drive.TransmissionSetLowGearCommand;
import frc.robot.commands.shooter.FlywheelSetVelocityCommand;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.TransmissionSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private DrivetrainSubsystem m_drivetrain = DrivetrainSubsystem.create();
  private TransmissionSubsystem m_transmission = TransmissionSubsystem.create();

  private FlywheelSubsystem m_flywheel = FlywheelSubsystem.create();
  private HoodSubsystem m_hood = HoodSubsystem.create();
  private TurretSubsystem m_turret = TurretSubsystem.create();

  private XboxController m_driveController = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DoubleSupplier moveSupplier = () -> {
      return m_driveController.getTriggerAxis(Hand.kRight) - m_driveController.getTriggerAxis(Hand.kLeft);
    };
    DoubleSupplier rotateSupplier = () -> {
      double rotate = m_driveController.getX(Hand.kLeft);
      if (Math.abs(rotate) < 0.1) {
        rotate = 0.0;
      }
      return rotate;
    };
    m_drivetrain.setDefaultCommand(new DrivetrainArcadeDriveCommand(m_drivetrain, moveSupplier, rotateSupplier));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureTransmissionBindings();

    new JoystickButton(m_driveController, XboxController.Button.kA.value).whenPressed(new RunCommand(() -> {
      double targetVoltage = 12.0
          * (m_driveController.getTriggerAxis(Hand.kRight) - m_driveController.getTriggerAxis(Hand.kLeft));
      m_flywheel.setVoltage(targetVoltage);
    }, m_flywheel));

    new JoystickButton(m_driveController, XboxController.Button.kBumperRight.value)
        .whenPressed(new FlywheelSetVelocityCommand(2000, m_flywheel));

    new JoystickButton(m_driveController, XboxController.Button.kX.value)
        .whenPressed(new RunCommand(m_flywheel::stop, m_flywheel));

    new JoystickButton(m_driveController, XboxController.Button.kY.value).whenPressed(new RunCommand(() -> {
      double currentPosition = m_hood.getPosition();
      double delta = -m_driveController.getY(Hand.kRight);
      if (Math.abs(delta) < 0.1) {
        delta = 0.0;
      }
      currentPosition += 0.5 * delta;
      m_hood.setPosition(currentPosition);
    }, m_hood));
  }

  private void configureTransmissionBindings() {
    new JoystickButton(m_driveController, XboxController.Button.kBumperLeft.value).whenPressed(new TransmissionSetLowGearCommand(m_transmission));
    new JoystickButton(m_driveController, XboxController.Button.kBumperRight.value).whenPressed(new TransmissionSetHighGearCommand(m_transmission));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("auto not implemented");
  }
}
