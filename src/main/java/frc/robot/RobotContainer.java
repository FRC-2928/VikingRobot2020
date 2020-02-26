package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.shooter.SpinUpFlywheel;
import frc.robot.oi.DriverOI;
import frc.robot.oi.impl.JettDriverOI;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.Limelight.Limelights;

public class RobotContainer {

  // The robot's subsystems
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
  }

  public void onInitialize(){
    m_flywheelsubsystem.configFeedbackGains();
    m_hoodsubsystem.configPIDGains();
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

    shooterVelocityControl.whileHeld(new RunCommand(() ->
    m_flywheelsubsystem.setFlywheelRPM(0), m_flywheelsubsystem));
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
}