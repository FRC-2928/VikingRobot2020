package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drive.DrivetrainArcadeDriveCommand;
import frc.robot.commands.drive.TransmissionSetHighGearCommand;
import frc.robot.commands.drive.TransmissionSetLowGearCommand;
import frc.robot.oi.DriverOI;
import frc.robot.oi.impl.JettDriverOI;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.climber.ElevatorSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.TransmissionSubsystem;
import frc.robot.subsystems.indexer.FeederSubsystem;
import frc.robot.subsystems.indexer.HopperSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;

public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrain;
  private final TransmissionSubsystem m_transmission;
  private final ElevatorSubsystem m_elevator;
  private final FlywheelSubsystem m_flywheel;
  private final HoodSubsystem m_hood;
  private final TurretSubsystem m_turret;
  private final FeederSubsystem m_feeder;
  private final HopperSubsystem m_hopper;

  private final Limelight m_shooterLimelight;
  private final DriverOI m_driverOI;

  public RobotContainer() {
    m_drivetrain = DrivetrainSubsystem.create();
    m_transmission = TransmissionSubsystem.create();
    m_elevator = ElevatorSubsystem.create();
    m_flywheel = FlywheelSubsystem.create();
    m_hood = HoodSubsystem.create();
    m_turret = TurretSubsystem.create();
    m_feeder = FeederSubsystem.create();
    m_hopper = HopperSubsystem.create();

    m_shooterLimelight = new Limelight(NetworkTableInstance.getDefault().getTable("limelight"));
    m_driverOI = new JettDriverOI(new XboxController(OIConstants.kDriveControllerPort));

    m_drivetrain.setDefaultCommand(
        new DrivetrainArcadeDriveCommand(m_drivetrain, m_driverOI.getMoveSupplier(), m_driverOI.getRotateSupplier()));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    configureDriveBindings();
    configureIndexerBindings();
    configureShooterBindings();
  }

  private void configureDriveBindings() {
    m_driverOI.getShiftLowButton().whenPressed(new TransmissionSetLowGearCommand(m_transmission));
    m_driverOI.getShiftHighButton().whenPressed(new TransmissionSetHighGearCommand(m_transmission));
  }

  private void configureIndexerBindings() {

  }

  private void configureShooterBindings() {

  }

  public Command getAutonomousCommand() {
    return new PrintCommand("auto not implemented");
  }
}
