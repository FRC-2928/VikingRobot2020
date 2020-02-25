package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auto.RamseteTrajectoryCommand;
import frc.robot.commands.shooter.SpinUpFlywheel;
import frc.robot.oi.DriverOI;
import frc.robot.oi.impl.JettDriverOI;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.TransmissionSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.trajectories.Test1Trajectory;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.Limelight.Limelights;

public class RobotContainer {

  // The robot's subsystems
  
  private final TransmissionSubsystem m_transmission = new TransmissionSubsystem();
  private final DrivetrainSubsystem m_robotDrive = new DrivetrainSubsystem(m_transmission::getGearState);
  private final FlywheelSubsystem m_flywheelsubsystem = new FlywheelSubsystem();
  private final HoodSubsystem m_hoodsubsystem = new HoodSubsystem();
  private final Limelight m_driverLimelight = new Limelight(Limelights.DRIVER);
  private final Limelight m_turretLimelight = new Limelight(Limelights.TURRET);

  private final XboxController driveController = new XboxController(0);

  private final JoystickButton hoodUp = new JoystickButton(driveController, 6);
  private final JoystickButton hoodDown = new JoystickButton(driveController, 5);
  private final JoystickButton hoodPosition = new JoystickButton(driveController, 1);
  private final JoystickButton shooterVelocityControl = new JoystickButton(driveController,4);
  private final JoystickButton turretOpenLoopLeft = new JoystickButton(driveController, 5);
  private final JoystickButton turretOpenLoopRight = new JoystickButton(driveController, 6);

  // private final JoystickButton openLoopFlywheel = new JoystickButton(driveController, 5);
  // private final JoystickButton velocityControlFlywheel = new JoystickButton(driveController, 6);
  // private final JoystickButton positionControlHood = new JoystickButton(driveController, 1);

  private final DriverOI m_driverOI;

  // Autonomous 
  // private Trajectory m_trajectory;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_driverOI = new JettDriverOI(new XboxController(OIConstants.kDriverControllerPort));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
      new RunCommand(() -> m_robotDrive
          .drive(m_driverOI.getMoveSupplier(),
                 m_driverOI.getRotateSupplier()), m_robotDrive)
    );
  }

  public void onInitialize(){
    m_flywheelsubsystem.configFeedbackGains();
    m_hoodsubsystem.configPIDGains();
    SmartDashboard.putNumber("Shooter Reference", 0);
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
    hoodUp.whileHeld(new RunCommand(m_hoodsubsystem::moveHoodUp, m_hoodsubsystem));
    hoodDown.whileHeld(new RunCommand(m_hoodsubsystem::moveHoodDown, m_hoodsubsystem));

    double reference = SmartDashboard.getNumber("Hood Reference", 0);
    hoodPosition.whileHeld(new RunCommand(() -> 
    m_hoodsubsystem.setHoodDegrees(reference), m_hoodsubsystem)
    );

    double shooterReference = SmartDashboard.getNumber("Shooter Reference", 0);
    shooterVelocityControl.whileHeld(new SpinUpFlywheel(m_flywheelsubsystem, shooterReference));
  }

  public void configureTurretButtons(){
  }

  public void configureControlPanelButtons() {
      
  }

  public void configureFeederButtons() {
    // Also need to pass in the flywheel
    //m_driverOI.getAutoShootingButton().whenPressed(new FastForwardFeeder(m_feeder));

  }

  public void ConfigureClimberButtons() {
  }


  private void configureIntakeButtons() {


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

    RamseteTrajectoryCommand trajectoryCommand = new RamseteTrajectoryCommand(m_robotDrive, trajectory1.getTrajectory());

    // Run path following command, then stop at the end.
    return trajectoryCommand.andThen(() -> m_robotDrive.stopDrivetrain());

  }
}