package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.commands.controlpanel.RotateSegments;
import frc.robot.commands.controlpanel.RotateToColor;
import frc.robot.subsystems.controlpanel.ControlPanelSubsystem;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.intake.FastForwardFeeder;
import frc.robot.commands.intake.StartFeeder;
import frc.robot.commands.intake.StopFeeder;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.oi.impl.AbbyOperatorOI;
import frc.robot.oi.impl.JettDriverOI;
import frc.robot.subsystems.intake.FeederSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.commands.climber.ClimbHigh;
import frc.robot.commands.climber.ClimbMid;
import frc.robot.commands.climber.DeployClimber;
import frc.robot.commands.climber.ClimbLow;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final DrivetrainSubsystem m_robotDrive = new DrivetrainSubsystem();

  private Trajectory m_trajectory;
  // The robot's subsystems and commands are defined here...
  private final FlywheelSubsystem m_flywheelsubsystem = new FlywheelSubsystem();
  private final HoodSubsystem m_hoodsubsystem = new HoodSubsystem();
  private final Intake m_intake = new Intake();
  private final ControlPanelSubsystem m_controlPanel = new ControlPanelSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  
  XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(Constants.OIConstants.kOperatorControllerPort);

  private final XboxController driveController = new XboxController(0);
  private final JoystickButton openLoopFlywheel = new JoystickButton(driveController, 5);
  private final JoystickButton velocityControlFlywheel = new JoystickButton(driveController, 6);
  private final JoystickButton positionControlHood = new JoystickButton(driveController, 1);
  // The robot's subsystems and commands are defined here...

  private final FeederSubsystem m_feeder = new FeederSubsystem();
 
  private final DriverOI m_driverOI;
  private final OperatorOI m_operatorOI;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_driverOI = new JettDriverOI(new XboxController(OIConstants.kDriverControllerPort));
    m_operatorOI = new AbbyOperatorOI(new XboxController(OIConstants.kOperatorControllerPort));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_robotDrive
            .drive(m_driverController.getY(GenericHID.Hand.kLeft),
                         m_driverController.getX(GenericHID.Hand.kRight)), m_robotDrive));
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
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kBumperRight.value)
        .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
        .whenReleased(() -> m_robotDrive.setMaxOutput(1));

    velocityControlFlywheel.whileHeld(
      new RunCommand(() -> {
        double targetRPM = SmartDashboard.getNumber("Target RPM", 0);
        m_flywheelsubsystem.setFlywheelRPM(targetRPM);
      }, 
      m_flywheelsubsystem));

    ConfigureControlButtons(); 

    openLoopFlywheel.whileHeld(new RunCommand(() -> m_flywheelsubsystem.setPower(0.75),m_flywheelsubsystem));

    positionControlHood.whileHeld(new RunCommand(() -> m_hoodsubsystem.setHoodDegrees(), m_hoodsubsystem));  
  }

  public void ConfigureControlButtons() {
      // Spin the control panel three times
      new JoystickButton(m_operatorController, Button.kY.value)
      .whenPressed(() -> m_controlPanel.rotateSegments(ControlPanelConstants.threeTurns));

    // Spin the control panel three times
    new JoystickButton(m_operatorController, Button.kY.value)
      .whenPressed(new RotateSegments(m_controlPanel, ControlPanelConstants.threeTurns));

    // Spin the control panel to target color
    new JoystickButton(m_operatorController, Button.kX.value)
      .whenPressed(new ConditionalCommand(

       // TRUE - the detected color is unknown so rotate half a segment 
       new RotateSegments(m_controlPanel, 0.5)
         // and then rotate to color
         .andThen(new RotateToColor(m_controlPanel)),

       // FALSE - the color is known so rotate to target color
       new RotateToColor(m_controlPanel),

       // CONDITION - is the color unknown?
       m_controlPanel.unknownColor()
     )

 );
  }

  public void onInitialize(){
    m_flywheelsubsystem.configFeedbackGains();
    m_hoodsubsystem.configPIDGains();
    //buttons for the intake
    // configureIntakeButtons();
    

    configureFeederButtons();
    
  }

  public void configureFeederButtons() {
    // Also need to pass in the flywheel
    //m_driverOI.getAutoShootingButton().whenPressed(new FastForwardFeeder(m_feeder));

    m_operatorOI.getEnableFeederButton().whenPressed(new StartFeeder(m_feeder));
    m_operatorOI.getDisableFeederButton().whenPressed(new StopFeeder(m_feeder));
    m_operatorOI.getReverseFeederButton().whileHeld(new RunCommand(() -> m_feeder.reverseFeeder()));
    ConfigureClimberButtons();
  
  }

  public void ConfigureClimberButtons() {
      
    // Deploy to top
    new JoystickButton(m_operatorController, Button.kA.value)
        .whenPressed(new DeployClimber(m_climber));

    // Deploy to high point
    new JoystickButton(m_operatorController, Button.kB.value)
        .whenPressed(new DeployClimber(m_climber)
          .andThen(new ClimbHigh(m_climber)));
     
    // Deploy to Mid point
    new JoystickButton(m_operatorController, Button.kX.value)
        .whenPressed(new DeployClimber(m_climber)
          .andThen(new ClimbMid(m_climber)));
      
    // Deploy to low point
    new JoystickButton(m_operatorController, Button.kY.value)
        .whenPressed(new DeployClimber(m_climber)
          .andThen(new ClimbLow(m_climber)));      
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
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

    String trajectoryJSON = "trajectories/test.wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    // Ramsete command will use PID within the TalonFX
    RamseteCommand ramseteCommand = new RamseteCommand(
        // Where we're going
        m_trajectory,

        // Where we are currently.  Input from the drivetrain
        m_robotDrive::getPose,

        // The controller. Computes the wheel speeds for the next spline
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
        
        // Kinematics, defined in the Constants file
        DrivetrainConstants.kDriveKinematics,

        // Consumed in the drivetrain to pass to PID loop
        m_robotDrive::outputMetersPerSecond,

        // Required subsystem
        m_robotDrive
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.stopDrivetrain());
  }
}