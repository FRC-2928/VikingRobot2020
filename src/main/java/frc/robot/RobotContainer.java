package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auto.DrivetrainCharacterizationCommand;
import frc.robot.commands.auto.ShootThreeThenDrive;
import frc.robot.commands.control.SetPositionCommand;
import frc.robot.commands.control.SetPowerCommand;
import frc.robot.commands.controlpanel.RotateSegments;
import frc.robot.commands.controlpanel.RotateToColor;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.intake.FastForwardFeeder;
import frc.robot.commands.intake.StartFeeder;
import frc.robot.commands.intake.StopFeeder;
import frc.robot.commands.shooter.SetHoodPosition;
import frc.robot.commands.shooter.SetSetpointShooting;
import frc.robot.commands.shooter.ShooterAtReference;
import frc.robot.commands.shooter.ShooterManagerSetReference;
import frc.robot.commands.shooter.SetSetpointShooting.ShooterSetpoint;
import frc.robot.commands.turret.TrackTargetCommand;
import frc.robot.commands.turret.TurretAtReference;
import frc.robot.commands.turret.TurretStopTracking;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.oi.impl.AbbiOperatorOI;
import frc.robot.oi.impl.JettDriverOI;
import frc.robot.subsystems.SubsystemContainer;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.controlpanel.ControlPanelSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.TransmissionSubsystem;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterManager;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.types.DistanceMap;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.Pigeon;
import frc.robot.utilities.Limelight.Limelights;

public class RobotContainer {
  private final SubsystemContainer m_subsystemContainer = new SubsystemContainer();

  private final SendableChooser<AutoType> m_autoChooser;

  public enum AutoType{
    DO_NOTHING, DRIVE, SHOOT_THEN_DRIVE;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_autoChooser = new SendableChooser<>();
    m_autoChooser.setDefaultOption("Do Nothing", AutoType.DO_NOTHING);
    m_autoChooser.addOption("Drive", AutoType.DRIVE);
    m_autoChooser.addOption("Shoot", AutoType.SHOOT_THEN_DRIVE);
    SmartDashboard.putData(m_autoChooser);

    JettDriverOI.bindDriverButtons(new XboxController(OIConstants.kDriverControllerPort), m_subsystemContainer);
    AbbiOperatorOI.bindOperatorButtons(new XboxController(OIConstants.kOperatorControllerPort), m_subsystemContainer);

    // Load the distance to target map
    DistanceMap.getInstance().loadMaps();

    // Configure the button bindings
    configureButtonBindings();
  }

  public void onAutoInit(){
    // new InstantCommand(m_pigeon::setStartConfig);
    new TrackTargetCommand(m_subsystemContainer.turret, m_subsystemContainer.drivetrain, m_subsystemContainer.turretLimelight).schedule();
  }

  public void onTeleopInit() {
    // new TrackTargetCommand(m_turret, m_drivetrain, m_turretLimelight).schedule();
    new StartFeeder(m_subsystemContainer.feeder).schedule();

    new ShooterManagerSetReference(m_subsystemContainer.shooterManager,
      () -> {
        //Hood
        if(m_subsystemContainer.turretLimelight.isTargetFound()){
          return m_subsystemContainer.distanceMap.getHoodDegrees(m_subsystemContainer.turretLimelight.getTargetDistance());
        }
        return m_subsystemContainer.distanceMap.getHoodDegrees(3);
      },
      () -> {
        //Flywheel
        if(m_subsystemContainer.turretLimelight.isTargetFound()){
          return m_subsystemContainer.distanceMap.getFlywheelRPM(m_subsystemContainer.turretLimelight.getTargetDistance());
        }
        return  m_subsystemContainer.distanceMap.getFlywheelRPM(5);
      }
      ).schedule();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    configureControlPanelButtons();
    configureFeederButtons();
    ConfigureClimberButtons();
    configureTurretButtons();
    configureShooterButtons();

  }

  public void configureShooterButtons(){

    m_operatorOI.getShootFromWallButton().whenPressed(
      new ShooterManagerSetReference(m_shooterManager, 0, 3500));

    m_operatorOI.getShootFromLineButton().whenPressed(
      new ShooterManagerSetReference(m_shooterManager, 35, 6100));
  }

  public void configureTurretButtons() {

    m_operatorOI.getEnableAutoTargetButton()
        .whenPressed(new TrackTargetCommand(m_turret, m_drivetrain, m_turretLimelight));

    m_operatorOI.getDisableAutoTargetButton()
        .whenPressed(new TurretStopTracking(m_turret));

    m_operatorOI.getMoveTurretButton()
        .whileHeld(new SetPowerCommand(m_turret, m_operatorOI.moveTurretSupplier()));
  }

  public void configureControlPanelButtons() {

    // // Spin the control panel three times
    // m_operatorOI.turnWheelButton().whileHeld(new SetPowerCommand(m_controlPanel, 0.4));

    // // Spin the control panel three times
    // m_operatorOI.turnWheelThreeTimes()
    //     .whenPressed(new RotateSegments(m_controlPanel, ControlPanelConstants.kRotationDistance));

    // // Spin the control panel to target color
    // m_operatorOI.turnToColorButton().whenPressed((new SetPositionCommand(m_turret, 0, true)));

    // m_operatorOI.turnToColorButton().whenReleased(new RotateToColor(m_controlPanel));
  }

  public void configureFeederButtons() {
    m_operatorOI.enableFeederButton().whenPressed(new StartFeeder(m_feeder));
    m_operatorOI.disableFeederButton().whenPressed(new StopFeeder(m_feeder));
    m_operatorOI.reverseFeederButton().whileHeld(new RunCommand(() -> m_feeder.reverseFeeder()));
    m_operatorOI.feedButton().whileHeld(new RunCommand(() -> m_feeder.fastForwardFeeder(), m_feeder));
  }

  public void ConfigureClimberButtons() {

    // if(m_operatorOI.deployToTop()){
    //   new DeployClimber(m_climber).schedule();
    // }


    // ((Button) m_operatorOI.lowerClimber()).whileHeld(new LowerClimber(m_climber));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    switch(m_autoChooser.getSelected()){
      case DO_NOTHING:
        return new WaitCommand(15);

      case DRIVE:
        return new Drive(m_subsystemContainer, 0.35, 0).withTimeout(2);

      case SHOOT_THEN_DRIVE:
        return new ShootThreeThenDrive(m_subsystemContainer);

      default:
        return new WaitCommand(15);
    }

    
    // // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(DrivetrainConstants.ksVolts,
    //                                    DrivetrainConstants.kvVoltSecondsPerMeter,
    //                                    DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
    //         DrivetrainConstants.kDriveKinematics,
    //         10);

    // //Create config for trajectory
    // TrajectoryConfig config =
    //   new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
    //                         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //       // Add kinematics to ensure max speed is actually obeyed
    //       .setKinematics(DrivetrainConstants.kDriveKinematics)
    //       // Apply the voltage constraint
    //       .addConstraint(autoVoltageConstraint);

    // // Get a trajectory
    // Test1Trajectory trajectory1 = new Test1Trajectory(config);

    // RamseteTrajectoryCommand trajectoryCommand = new RamseteTrajectoryCommand(m_drivetrain, trajectory1.getTrajectory());

    // // Run path following command, then stop at the end.
    // return trajectoryCommand.andThen(() -> m_drivetrain.stopDrivetrain());
  }
}