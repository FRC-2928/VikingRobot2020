package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auto.AutoFeedCommand;
import frc.robot.commands.auto.AutoIndexCommand;
import frc.robot.commands.auto.AutoTargetCommand;
import frc.robot.commands.drive.DrivetrainArcadeDriveCommand;
import frc.robot.commands.drive.TransmissionSetHighGearCommand;
import frc.robot.commands.drive.TransmissionSetLowGearCommand;
import frc.robot.oi.DriverOI;
import frc.robot.oi.impl.JettDriverOI;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.TransmissionSubsystem;
import frc.robot.subsystems.indexer.FeederSubsystem;
import frc.robot.subsystems.indexer.HopperSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.utilities.LimeliteUtility;

public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrain;
  private final TransmissionSubsystem m_transmission;
  private final FlywheelSubsystem m_flywheel;
  private final HoodSubsystem m_hood;
  private final TurretSubsystem m_turret;
  private final FeederSubsystem m_feeder;
  private final HopperSubsystem m_hopper;

  private final DriverOI m_driverOI;

  public RobotContainer() {
    m_drivetrain = DrivetrainSubsystem.create();
    m_transmission = TransmissionSubsystem.create();
    m_flywheel = FlywheelSubsystem.create();
    m_hood = HoodSubsystem.create();
    m_turret = TurretSubsystem.create();
    m_feeder = FeederSubsystem.create();
    m_hopper = HopperSubsystem.create();

    m_driverOI = new JettDriverOI(new XboxController(OIConstants.kDriveControllerPort));

    m_drivetrain.setDefaultCommand(new DrivetrainArcadeDriveCommand(m_drivetrain, m_driverOI.getMoveSupplier(), m_driverOI.getRotateSupplier()));

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
    m_driverOI.getFlywheelButton().whenPressed(new AutoFeedCommand(m_hopper, m_feeder, () -> {
      // This method checks that hood, turret and flywheel are all near their
      // setpoints. The feeder will only run if they are all on target.
      //return m_hood.atTargetPosition() && m_turret.atTargetPosition() && m_flywheel.atTargetVelocity();

      return true;
    }));
    m_driverOI.getFlywheelButton().whenReleased(new AutoIndexCommand(m_hopper, m_feeder));
  }

  private void configureShooterBindings() {
    m_driverOI.getEnableAutoTargetButton().whenPressed(new AutoTargetCommand(m_hood,
        m_turret, m_flywheel, LimeliteUtility::getData, m_drivetrain::getHeading, m_driverOI.getFlywheelButton()));

    m_driverOI.getEnableAutoTargetButton().whenPressed(new InstantCommand(() -> {
      m_hopper.getCurrentCommand().cancel();
      m_turret.getCurrentCommand().cancel();
      m_flywheel.getCurrentCommand().cancel();
    }));
  }

  public Command getAutonomousCommand() {
    return new PrintCommand("auto not implemented");
  }
}
