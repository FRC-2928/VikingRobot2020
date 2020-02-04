package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.org.ballardrobotics.triggers.AxisNonZeroTrigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auto.AutoTargetCommand;
import frc.robot.commands.drive.DrivetrainArcadeDriveCommand;
import frc.robot.commands.drive.TransmissionSetHighGearCommand;
import frc.robot.commands.drive.TransmissionSetLowGearCommand;
import frc.robot.commands.shooter.FlywheelSetVelocityCommand;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.TransmissionSubsystem;
import frc.robot.subsystems.indexer.FeederSubsystem;
import frc.robot.subsystems.indexer.HopperSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.utilities.LimeliteUtility;
import frc.robot.utilities.MathUtility;

public class RobotContainer {
  private DrivetrainSubsystem m_drivetrain = DrivetrainSubsystem.create();
  private TransmissionSubsystem m_transmission = TransmissionSubsystem.create();

  private FlywheelSubsystem m_flywheel = FlywheelSubsystem.create();
  private HoodSubsystem m_hood = HoodSubsystem.create();
  private TurretSubsystem m_turret = TurretSubsystem.create();

  private FeederSubsystem m_feeder = FeederSubsystem.create();
  private HopperSubsystem m_hopper = HopperSubsystem.create();

  private XboxController m_driveController = new XboxController(OIConstants.kDriveControllerPort);

  public RobotContainer() {
    DoubleSupplier moveSupplier = () -> m_driveController.getTriggerAxis(Hand.kRight)
        - m_driveController.getTriggerAxis(Hand.kLeft);
    DoubleSupplier rotateSupplier = () -> MathUtility.applyDeadband(m_driveController.getX(Hand.kLeft), 0.1);
    m_drivetrain.setDefaultCommand(new DrivetrainArcadeDriveCommand(m_drivetrain, moveSupplier, rotateSupplier));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    configureDriveBindings();
    configureIndexerBindings();
    configureShooterBindings();
  }

  private void configureDriveBindings() {
    new JoystickButton(m_driveController, XboxController.Button.kBumperLeft.value)
        .whenPressed(new TransmissionSetLowGearCommand(m_transmission));
    new JoystickButton(m_driveController, XboxController.Button.kBumperRight.value)
        .whenPressed(new TransmissionSetHighGearCommand(m_transmission));
  }

  private void configureIndexerBindings() {
    new JoystickButton(m_driveController, XboxController.Button.kStart.value)
        .whenPressed(new AutoTargetCommand(m_hood, m_turret, LimeliteUtility::getData, m_drivetrain::getHeading));
  }

  private void configureShooterBindings() {
    new AxisNonZeroTrigger(() -> m_driveController.getTriggerAxis(Hand.kRight)).whileActiveContinuous(new PrintCommand("trigger pressed"));
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("auto not implemented");
  }
}
