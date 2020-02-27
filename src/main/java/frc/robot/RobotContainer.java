package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.trajectories.Test1Trajectory;
import frc.robot.types.DistanceMap;
import frc.robot.types.LimelightData;
import frc.robot.types.TargetEstimate;
import frc.robot.commands.controlpanel.RotateSegments;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;
import frc.robot.utilities.Limelight;
import frc.robot.commands.controlpanel.RotateToColor;
import frc.robot.commands.turret.TrackTargetCommand;
import frc.robot.commands.turret.TurretLimelightSetPosition;
import frc.robot.commands.turret.TurretSetStateCommand;
import frc.robot.subsystems.controlpanel.ControlPanelSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auto.RamseteTrajectoryCommand;
import frc.robot.commands.climber.DeployClimber;
import frc.robot.commands.control.SetPositionCommand;
import frc.robot.commands.control.SetPowerCommand;
import frc.robot.commands.intake.FastForwardFeeder;
import frc.robot.commands.intake.StartFeeder;
import frc.robot.commands.intake.StopFeeder;
import frc.robot.commands.shooter.SetHoodPosition;
import frc.robot.commands.shooter.SpinUpFlywheel;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.oi.impl.AbbiOperatorOI;
import frc.robot.oi.impl.JettDriverOI;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.TransmissionSubsystem;
import frc.robot.subsystems.intake.FeederSubsystem;
import frc.robot.utilities.Limelight.Limelights;

public class RobotContainer {

  // The robot's subsystems
  
  private final TransmissionSubsystem m_transmission = new TransmissionSubsystem();
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem(m_transmission::getGearState);
  private final FlywheelSubsystem m_flywheelsubsystem = new FlywheelSubsystem();
  private final HoodSubsystem m_hoodsubsystem = new HoodSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ControlPanelSubsystem m_controlPanel = new ControlPanelSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();
  private final Limelight m_driverLimelight = new Limelight(Limelights.DRIVER);
  private final Limelight m_turretLimelight = new Limelight(Limelights.TURRET);

  private final XboxController driveController = new XboxController(0);

  private final JoystickButton turretPositionControl = new JoystickButton(driveController, 1);
  private final JoystickButton turretFieldCentricControl = new JoystickButton(driveController, 2);
  private final JoystickButton turretVisionControl = new JoystickButton(driveController, 3);
  private final JoystickButton turretOpenLoopLeft = new JoystickButton(driveController, 5);
  private final JoystickButton turretOpenLoopRight = new JoystickButton(driveController, 6);

  // private final JoystickButton openLoopFlywheel = new JoystickButton(driveController, 5);
  // private final JoystickButton velocityControlFlywheel = new JoystickButton(driveController, 6);
  // private final JoystickButton positionControlHood = new JoystickButton(driveController, 1);

  private final DriverOI m_driverOI;
  private final OperatorOI m_operatorOI;

  // Autonomous 
  // private Trajectory m_trajectory;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

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
      new RunCommand(() -> m_drivetrain
          .drive(m_driverOI.getMoveSupplier(),
                 m_driverOI.getRotateSupplier()), m_drivetrain)
    );
  }

  public void onInitialize(){
    // m_flywheelsubsystem.configPIDGains();
    // m_hoodsubsystem.configPIDGains();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   *
   * 
   */
  private void configureButtonBindings() {

    configureControlPanelButtons(); 
    configureIntakeButtons();
    configureFeederButtons();
    ConfigureClimberButtons();
    configureTurretButtons();
    configureDrivetrainButtons();

    // Set the hood and flywheel
    m_driverOI.setShooterButton().whenPressed(new ParallelCommandGroup(
      new SetHoodPosition(m_hoodsubsystem, m_turretLimelight),
      new SpinUpFlywheel(m_flywheelsubsystem, m_turretLimelight)));

    m_driverOI.getAutoShootingButton().whileHeld(new FastForwardFeeder(m_feeder, m_hoodsubsystem, m_flywheelsubsystem)); 
  }

  public void configureDrivetrainButtons() {
    m_driverOI.getShiftLowButton()
    .whenPressed(new InstantCommand(m_transmission::setLow, m_transmission));

    m_driverOI.getShiftHighButton()
    .whenPressed(new InstantCommand(m_transmission::setHigh, m_transmission));
  }

  public void configureTurretButtons(){
    // turretVisionControl.whileHeld(new TurretSetStateCommand(
    //   m_turret, TurretControlState.VISION_TRACKING, 0, new TargetEstimate(0, 0, false)));

    m_operatorOI.getEnableAutoTargetButton()
      .whenPressed(new TrackTargetCommand(m_turret, m_drivetrain, new TargetEstimate(0, 0, false)));

    turretOpenLoopLeft.whileHeld(new SetPowerCommand(m_turret, 0.4));
    turretOpenLoopRight.whileHeld(new SetPowerCommand(m_turret, -0.4));
    turretPositionControl.whileHeld(new SetPositionCommand(m_hoodsubsystem, 30));
  }

  public void configureControlPanelButtons() {
      
    // Spin the control panel three times
    m_operatorOI.turnWheelButton()
      .whileHeld(new SetPowerCommand(m_controlPanel, 0.4));

      // m_operatorOI.turnWheelButton()
      // .whileHeld(new RunCommand(m_controlPanel::setPower, m_controlPanel));

    // Spin the control panel three times
    m_operatorOI.turnWheelThreeTimes()
      .whenPressed(new RotateSegments(m_controlPanel, ControlPanelConstants.kRotationDistance));

    // Spin the control panel to target color
    m_operatorOI.turnToColorButton()
      .whenPressed(new ConditionalCommand(
       // TRUE - the detected color is unknown so rotate half a segment 
       new RotateSegments(m_controlPanel, 0.5)
         // and then rotate to color
         .andThen(new RotateToColor(m_controlPanel)),
       // FALSE - the color is known so rotate to target color
       new RotateToColor(m_controlPanel),
       // CONDITION - is the color unknown?
       m_controlPanel.unknownColor()
     ));
  }

  public void configureFeederButtons() {
    // Also need to pass in the flywheel
    //m_driverOI.getAutoShootingButton().whenPressed(new FastForwardFeeder(m_feeder));

    m_operatorOI.enableFeederButton().whenPressed(new StartFeeder(m_feeder));
    m_operatorOI.disableFeederButton().whenPressed(new StopFeeder(m_feeder));
    m_operatorOI.reverseFeederButton().whileHeld(new RunCommand(() -> m_feeder.reverseFeeder()));
  }

  public void ConfigureClimberButtons() {
  
    m_operatorOI.deployToTop()
      .whenPressed(new DeployClimber(m_climber));  
  }


  private void configureIntakeButtons() {

    // Pickup balls from the ground
    m_driverOI.getGroundIntakeButton().whenPressed(new SequentialCommandGroup(
      //extend intake
      new InstantCommand(m_intake::groundPickup, m_intake ),
      //wait until intake deploys
      new WaitCommand(0.5),
      // run motors
      new RunCommand(m_intake::startMotor, m_intake)
    ));

    // Pickup balls from the Player Station
    m_driverOI.getStationIntakeButton().whenPressed(new SequentialCommandGroup(
      //extend intake
      new InstantCommand(m_intake::stationPickup, m_intake ),
      //wait until intake deploys
      new WaitCommand(0.5),
      // run motors
      new RunCommand(m_intake::startMotor, m_intake)
    ));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DrivetrainConstants.ksVolts,
                                       DrivetrainConstants.kvVoltSecondsPerMeter,
                                       DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
            DrivetrainConstants.kDriveKinematics,
            10);

    //Create config for trajectory
    TrajectoryConfig config =
      new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DrivetrainConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);

    // Get a trajectory
    Test1Trajectory trajectory1 = new Test1Trajectory(config);

    RamseteTrajectoryCommand trajectoryCommand = new RamseteTrajectoryCommand(m_drivetrain, trajectory1.getTrajectory());

    // Run path following command, then stop at the end.
    return trajectoryCommand.andThen(() -> m_drivetrain.stopDrivetrain());

  }
}