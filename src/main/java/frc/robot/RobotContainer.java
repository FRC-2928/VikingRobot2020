/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import frc.robot.commands.controlpanel.RotateToColor;
import frc.robot.subsystems.controlpanel.ControlPanelSubsystem;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


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
  
  XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(Constants.OIConstants.kOperatorControllerPort);

  private final XboxController driveController = new XboxController(0);
  private final JoystickButton openLoopFlywheel = new JoystickButton(driveController, 5);
  private final JoystickButton velocityControlFlywheel = new JoystickButton(driveController, 6);
  private final JoystickButton positionControlHood = new JoystickButton(driveController, 1);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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

        // Spin the control panel to target color
        //may switch to have one button control all.
    new JoystickButton(m_operatorController, Button.kX.value)
    .whenPressed(new ConditionalCommand(

       // TRUE - the detected color is unknown
       new RotateToColor(m_controlPanel)
         // so rotate half a segment before rotating to color
         .beforeStarting(m_controlPanel::rotateHalfSegment),

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
    configureIntakeButtons();
    
  }

  private void configureIntakeButtons() {
    // Pickup balls from the ground
    new JoystickButton(m_driverController, Button.kA.value).whenPressed(new SequentialCommandGroup(
      //extend intake
      new InstantCommand(m_intake::groundPickup, m_intake ),
      //wait until intake deploys
      new WaitCommand(1),
      // run motors
      new RunCommand(m_intake::startMotor, m_intake)
    ));


    // Stow the intake
    new JoystickButton(m_driverController, Button.kB.value).whenReleased(new SequentialCommandGroup(
      //stop motors
      new InstantCommand(m_intake::stopMotor, m_intake),
      //retract intake
      new InstantCommand(m_intake::Stowed, m_intake )
    ));



    // Pickup balls from the Player Station
    new JoystickButton(m_driverController, Button.kX.value).whenPressed(new SequentialCommandGroup(
      //extend intake
      new InstantCommand(m_intake::StationPickup, m_intake ),
      //wait until intake deploys
      new WaitCommand(1),
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
