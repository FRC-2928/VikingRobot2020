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
import frc.robot.commands.shooter.SetShooter;
import frc.robot.commands.shooter.ShooterAtReference;
import frc.robot.commands.shooter.ShooterManagerSetReference;
import frc.robot.commands.shooter.SpinUpFlywheel;
import frc.robot.commands.shooter.SetShooter.ShooterSetpoint;
import frc.robot.commands.turret.TrackTargetCommand;
import frc.robot.commands.turret.TurretAtReference;
import frc.robot.commands.turret.TurretStopTracking;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.oi.impl.AbbiOperatorOI;
import frc.robot.oi.impl.JettDriverOI;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.controlpanel.ControlPanelSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.TransmissionSubsystem;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterManager;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.types.DistanceMap;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.Pigeon;
import frc.robot.utilities.Limelight.Limelights;

public class RobotContainer {

  // The robot's subsystems

  private final TransmissionSubsystem m_transmission = new TransmissionSubsystem();
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem(m_transmission::getGearState);
  private final FlywheelSubsystem m_flywheel = new FlywheelSubsystem();
  private final HoodSubsystem m_hood = new HoodSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  // private final ControlPanelSubsystem m_controlPanel = new ControlPanelSubsystem();
  // private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();
  private final Pigeon m_pigeon = new Pigeon();
  private final ShooterManager m_shooterManager = new ShooterManager();
  private final Limelight m_driverLimelight = new Limelight(Limelights.DRIVER);
  private final Limelight m_turretLimelight = new Limelight(Limelights.TURRET);

  private final DriverOI m_driverOI;
  private final OperatorOI m_operatorOI;

  private final DistanceMap m_distanceMap = DistanceMap.getInstance();

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

    m_driverOI = new JettDriverOI(new XboxController(OIConstants.kDriverControllerPort));
    m_operatorOI = new AbbiOperatorOI(new XboxController(OIConstants.kOperatorControllerPort));

    // Load the distance to target map
    DistanceMap.getInstance().loadMaps();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_drivetrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_drivetrain.drive(m_driverOI.getMoveSupplier(), m_driverOI.getRotateSupplier()),
            m_drivetrain));
  }

  public void onAutoInit(){
    // new InstantCommand(m_pigeon::setStartConfig);
    new TrackTargetCommand(m_turret, m_drivetrain, m_turretLimelight).schedule();
  }

  public void onTeleopInit() {
    new TrackTargetCommand(m_turret, m_drivetrain, m_turretLimelight).schedule();
    new StartFeeder(m_feeder).schedule();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    configureControlPanelButtons();
    configureIntakeButtons();
    configureFeederButtons();
    ConfigureClimberButtons();
    configureTurretButtons();
    configureShooterButtons();
    configureDrivetrainButtons();

  }

  public void configureDrivetrainButtons() {
    m_driverOI.getShiftLowButton().whenPressed(new InstantCommand(m_transmission::setLow, m_transmission));

    m_driverOI.getShiftHighButton().whenPressed(new InstantCommand(m_transmission::setHigh, m_transmission));
  }

  public void configureShooterButtons(){
    // Set the hood and flywheel
    // m_driverOI.getAutoShootingButton()
    //     .whileHeld(new ParallelCommandGroup(
    //       new SetHoodPosition(m_hood, m_turretLimelight),
    //       new SpinUpFlywheel(m_flywheel, m_turretLimelight))
    // );
    String flywheelKey = "Shooter Manager Flywheel Reference";
    String hoodKey = "Shooter Manager Hood Reference";
    SmartDashboard.putNumber(hoodKey, 35);
    SmartDashboard.putNumber(flywheelKey, 4250);

    // m_driverOI.getAutoShootingButton().whileHeld(
    //   new ParallelCommandGroup(
    //     new RunCommand(() -> m_flywheel.setVelocity(m_shooterManager.getFlywheelReference()), m_flywheel),
    //     new RunCommand(() -> m_hood.setHoodDegrees(m_shooterManager.getHoodReference()), m_hood)
    //   )
    // );
    
    m_driverOI.getAutoShootingButton().whileHeld(
      new RunCommand(
        () -> {
            m_flywheel.setVelocity(m_shooterManager.getFlywheelReference());
            m_hood.setHoodDegrees(m_shooterManager.getHoodReference());
        }, 
        m_flywheel,m_hood));

    m_driverOI.getShooterDebugButton().whenPressed(
      new ShooterManagerSetReference(
        m_shooterManager,
        () -> SmartDashboard.getNumber(hoodKey, 0),
        () -> SmartDashboard.getNumber(flywheelKey, 0)
    ));

    m_driverOI.getFenderShotButton().whenPressed(
      new ShooterManagerSetReference(m_shooterManager,
      () -> {
        //Hood
        if(m_turretLimelight.isTargetFound()){
          return m_distanceMap.getHoodDegrees(m_turretLimelight.getTargetDistance());
        }
        return 0;
      },
      () -> {
        //Flywheel
        if(m_turretLimelight.isTargetFound()){
          return m_distanceMap.getFlywheelRPM(m_turretLimelight.getTargetDistance());
        }
        return 3000;
      }
      ));

    m_driverOI.getInitiationlineShotButton().whenPressed(
      new ShooterManagerSetReference(m_shooterManager, 35, 4000)); //4000
    

    m_driverOI.getFeedButton().whileHeld(new RunCommand(m_feeder::startFeeder, m_feeder));
    m_driverOI.getFeedButton().whenReleased(new StartFeeder(m_feeder));

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

  private void configureIntakeButtons() {

    // Pickup balls from the ground
    m_driverOI.getGroundIntakeButton().whileHeld(new SequentialCommandGroup(
      //extend intake
      new InstantCommand(m_intake::groundPickup, m_intake),
      //wait until intake deploys
      // new WaitCommand(0.1),
      // run motors
      new RunCommand(m_intake::startMotor, m_intake)
    ));

    // Pickup balls from the Player Station
    m_driverOI.getStationIntakeButton().whileHeld(new SequentialCommandGroup(
      //extend intake
      new InstantCommand(m_intake::stationPickup, m_intake ),
      //wait until intake deploys
      // new WaitCommand(0.1),
      // run motors
      new RunCommand(m_intake::reverseMotor, m_intake)
    ));
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
        return new Drive(m_drivetrain, 0.35, 0).withTimeout(2);

      case SHOOT_THEN_DRIVE:
        return new ShootThreeThenDrive(m_drivetrain, m_flywheel, m_hood, m_turret, m_feeder, m_turretLimelight);

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